//
// Created by zlc on 2021/4/11.
//

#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <chrono>
#include <ctime>
#include <climits>      // 定义的符号常量,比如FLT_MIN 　　　　　　　保留全部精度的float类型正数最小值
                        // FLT_MAX　　　　　　　 float类型正数最大值

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>          // cv::cvtColor()
#include <opencv2/highgui/highgui.hpp>          // cv::imread()
#include <opencv2/features2d/features2d.hpp>


#include <g2o/core/base_unary_edge.h>           // 边
#include <g2o/types/sba/types_six_dof_expmap.h>     // 含顶点VertexSE3Expmap
#include <g2o/core/block_solver.h>              // 块求解
#include <g2o/core/optimization_algorithm_levenberg.h>      // 求解△x，这里使用(H+△I)△x = g
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>




using namespace std;
using namespace g2o;


/* +++++++++++++++++++++++++++参数配置区+++++++++++++++++++++++++++++++++++++++ */

// 下面两种配置参数 可以任意组合 互不冲突 下面列出4种情况:
/*  SEMIDENSE    BookExercise
 *  0            0           : 采用direct_sparse 例子 稀疏直接法（提取FAST特征）
 *  0            1           : 采用书上课后习题3（小块均值法）+ 稀疏直接法（提取FAST特征）
 *  1            0           : 采用稀疏直接法(提取FAST特征) + 半稠密直接法（选取梯度大的）
 *  1            1           : 采用半稠密直接法（选取梯度大的）+课后习题3思路（小块均值法）+稀疏直接法（提取FAST特征）
 *  从枚举器中选择合适的值，对应修改下面的两个值:
 *  GradientThread             梯度域值
 *  PATCH_RADIUS               小块半径
 *  */
#define  SEMIDENSE       0       // 1 (稀疏直接法+半稠密法) 表示利用半稠密方法思想 筛选梯度比较大的点  这里就是从FAST关键点中筛选出梯度大的点
                                 // 0 表示仅仅用稀疏直接法
#define  BookExercise    0       // 1 表示运行课后习题3的做法
                                 // 0 表示不采用课后习题3的做法

#if SEMIDENSE
enum GradientThreadChoice
{
    GRADIENT_10 = 10,       //1170点
    GRADIENT_15 = 15,       //984点
    GRADIENT_20 = 20,       //805点
    GRADIENT_25 = 25,       //656点
    GRADIENT_30 = 30,       //514点
    GRADIENT_50 = 50,       //201点
    GRADIENT_100 = 100      //33点
};
#define  GradientThread  GRADIENT_10    // 默认筛选的梯度域值,通过调节域值（默认50） 可以增加关键点的个数
#endif

#if BookExercise
enum PatchRadiusChoices     // 块大小选取类
{
    PATCH_RADIUS_ONE = 1,   // 表示以像素为圆心，半径为1大小的块
    PATCH_RADIUS_TWO = 2    // 最多半径为2，否则计算量太大（因为边计算误差函数会进行插值查找，块越大，计算量成平方增加）
};
PatchRadiusChoices PATCH_RADIUS = PATCH_RADIUS_TWO;     // 将全局变量置为该选项半径为1,1对应3×3小块，2对应5×5小块
#endif

#if BookExercise
float getPatchAverageGray(const cv::Mat& gray, float u, float v, int patchRadius);
// 一次测量的值，包括一个世界坐标系下三维点（以第一帧为参考系）与一个灰度值（以第一帧为参考的3D点对应灰度图像的灰度值，
// 灰度图是由color图像转换到对应的gray图像得到的 ）
#endif


/********************************************
 * 本节演示了RGBD上的稀疏直接法
 ********************************************/

// 一次测量的值，包括一个世界坐标系下三维点与一个灰度值
struct Measurement
{
    Eigen::Vector3d pos_world;
    float grayscale;
    Measurement ( Eigen::Vector3d p, float g ) : pos_world( p ), grayscale( g ) {  }
};

// 像素坐标系转相机坐标系
inline Eigen::Vector3d project2Dto3D ( int x, int y, int d, float fx, float fy, float cx, float cy, float scale )
{
    float zz = float( d )  / scale;
    float xx = zz * (x-cx) / fx;
    float yy = zz * (y-cy) / fy;

    return Eigen::Vector3d(xx, yy, zz);
}

/*
 * inline是C++语言中的一个关键字，可以用于程序中定义内联函数，inline的引进使内联函数的定义更加简单。
 * 说到内联函数，这里给出比较常见的定义，内联函数是C++中的一种特殊函数，它可以像普通函数一样被调用，
 * 但是在调用时并不通过函数调用的机制而是通过将函数体直接插入调用处来实现的，这样可以大大减少由函数调用带来的开销，
 * 从而提高程序的运行效率。一般来说inline用于定义类的成员函数。
 * */
// 相机坐标系转像素坐标系
inline Eigen::Vector2d project3Dto2D ( float x, float y, float z, float fx, float fy, float cx, float cy )
{
    float u = fx * x/z + cx;
    float v = fy * y/z + cy;

    return Eigen::Vector2d(u, v);
}

// 直接法估计位姿
// 输入：测量值（空间点的灰度），新的灰度图，相机内参；输出：相机位姿
// 返回：true为成功，false维失败
bool poseEstimationDirect ( const vector<Measurement>& measurements, cv::Mat* gray, Eigen::Matrix3f& K, Eigen::Isometry3d& Tcw );

// project a 3d point into an image plane, the error is photometric error
// an unary edge with one vertex SE3Expmap (the pose of camera)
                                          // 误差(观测)值维度：这里是1，因为误差只有一个灰度值 误差类型   顶点类型
class EdgeSE3ProjectDirect : public BaseUnaryEdge< 1, double, VertexSE3Expmap >
{
public:
    Eigen::Vector3d x_world_;           // 3D point in world frame  世界坐标系帧下的3D点
    float cx_=0, cy_=0, fx_=0, fy_=0;   // Camera intrinsics
    cv::Mat* image_ = nullptr;          // reference image


public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // EdgeSE3ProjectDirect() = default;       // 代替下面的方式，默认产生合成的构造函数
    EdgeSE3ProjectDirect() {  }
    EdgeSE3ProjectDirect ( Eigen::Vector3d point, float fx, float fy, float cx, float cy, cv::Mat* image )
        : x_world_ ( point ), fx_( fx ), fy_(fy), cx_(cx), cy_(cy), image_(image)       // 灰度图形指针
    {
    }

    // 第一步：计算灰度残差e()
    virtual void computeError() override
    {
        // 强制类型转换
        const VertexSE3Expmap* v = static_cast<const VertexSE3Expmap*> ( _vertices[0] );
        Eigen::Vector3d x_local = v->estimate().map( x_world_ );    // 世界坐标系转换为当前相机坐标系
        float x = x_local[0] * fx_ / x_local[2] + cx_;              // 当前相机坐标系转换到当前帧像素坐标系
        float y = x_local[1] * fy_ / x_local[2] + cy_;

        // check x,y is in the image
        // 距离图像四条边4个像素大小的区域内作为有效投影区域
        // 对于不在该范围内的点误差值设为0 为了防止计算的误差太大 拉低内点对误差的影响 导致估计的RT严重偏离真值
        if ( x-4<0 || (x+4)>image_->cols || (y-4)<0 || (y+4)>image_->rows )
        {
            _error(0,0) = 0.0;
            this->setLevel(1);          // ???????????????????????????????????????????????????
        }
        else
        {
#if BookExercise    // 表示运行课后习题3
            // 选取小块的大小为2×2
            float sumValue = 0.0;
            for ( int i = x-PATCH_RADIUS; i<= x+PATCH_RADIUS; ++ i )
            {
                for (int j = y-PATCH_RADIUS; j <= y+PATCH_RADIUS; ++ j)
                {
                    sumValue += getPixelValue(i, j);                // 通过双线性差值获取浮点坐标对应的插值后的灰度像素值
                }
            }
            sumValue /= ( (2*PATCH_RADIUS+1)*(2*PATCH_RADIUS+1) );  // 求得元素周围小块的平均灰度值
            _error(0,0) = sumValue - _measurement;
#else
            _error(0,0) = getPixelValue( x,y ) - _measurement;      // 经过在灰度图中插值获得的像素值 减去测量值
#endif
        }
    }

    // 第二步：计算雅克比矩阵， 计算 eplus in manifold
    // 提供误差关于位姿的雅克比矩阵 书上8.16式子 只不过负号去掉了 因为用的是当前帧灰度值 - 世界坐标下的测量值
    virtual void linearizeOplus( ) override
    {
        if ( 1 == level() )
        {
            _jacobianOplusXi = Eigen::Matrix<double, 1, 6>::Zero();
            return ;
        }
        VertexSE3Expmap* vtx = dynamic_cast<VertexSE3Expmap*> (_vertices[0]);
        Eigen::Vector3d xyz_trans = vtx->estimate().map( x_world_ );        // q in book 将世界坐标系下的点，转换到第二帧坐标系下

        double x = xyz_trans[0];            // X
        double y = xyz_trans[1];            // Y
        double invz = 1.0 / xyz_trans[2];   // 1/Z
        double invz_2 = invz * invz;        // 1/Z²

        float u = x*fx_*invz + cx_;         // 投影到第二帧的像素坐标系下
        float v = y*fy_*invz + cy_;

        // jacobian from se3 to u,v
        // NOTE that in g2o the Lie algebra is (\omega, \epsilon), where \omega is so(3) and \epsilon the translation
        Eigen::Matrix<double, 2, 6> jacobian_uv_ksai;

        // 书上8.15式子，旋转在前，平移在后
        jacobian_uv_ksai ( 0,0 ) = - x*y*invz_2 *fx_;
        jacobian_uv_ksai ( 0,1 ) = ( 1+ ( x*x*invz_2 ) ) *fx_;
        jacobian_uv_ksai ( 0,2 ) = - y*invz *fx_;
        jacobian_uv_ksai ( 0,3 ) = invz *fx_;
        jacobian_uv_ksai ( 0,4 ) = 0;
        jacobian_uv_ksai ( 0,5 ) = -x*invz_2 *fx_;

        jacobian_uv_ksai ( 1,0 ) = - ( 1+y*y*invz_2 ) *fy_;
        jacobian_uv_ksai ( 1,1 ) = x*y*invz_2 *fy_;
        jacobian_uv_ksai ( 1,2 ) = x*invz *fy_;
        jacobian_uv_ksai ( 1,3 ) = 0;
        jacobian_uv_ksai ( 1,4 ) = invz *fy_;
        jacobian_uv_ksai ( 1,5 ) = -y*invz_2 *fy_;

        Eigen::Matrix<double, 1, 2> jacobian_pixel_uv;

        // 书上I2对像素坐标系的偏导数  这里很有可能 计算出来的梯度为0  因为FAST角点的梯度没有限制
        // 这也是半稠密法主要改进的地方 就是选关键点的时候 选择梯度大的点 因此这里的梯度就不可能为0了
        jacobian_pixel_uv ( 0,0 ) = ( getPixelValue ( u+1,v )-getPixelValue ( u-1,v ) ) /2;
        jacobian_pixel_uv ( 0,1 ) = ( getPixelValue ( u,v+1 )-getPixelValue ( u,v-1 ) ) /2;

        _jacobianOplusXi = jacobian_pixel_uv*jacobian_uv_ksai;//书上8.16式子

    }


    // dummy read and write functions because we don't care...
    virtual bool read ( std::istream& in) {  }
    virtual bool write ( std::ostream& out ) const {  }

protected:
    // get a gray scale value from reference image (bilinear interpolated)
    // cv::Mat中成员变量代表的含义:http://blog.csdn.net/dcrmg/article/details/52294259
    // 下面的方式 针对单通道的灰度图
    inline float getPixelValue ( float x, float y )     // 通过双线性差值获取浮点坐标对应的插值后的像素值
    {
        uchar* data = & image_->data[ int(y) * image_->step + int(x) ]; // step 表示图像矩阵一行的所有字节（包括所有通道的总和），data表示存储图像的起始指针
        float xx = x - floor(x);
        float yy = y - floor(y);                        // 向下取整函数

        return float (  // 公式f(i+u,j+v) = (1-u)(1-v)f(i,j) + u(1-v)f(i+1,j) + (1-u)vf(i,j+1) + uvf(i+1,j+1)
                        // 这里的xx 就是u  yy就是v
                ( 1-xx ) * ( 1-yy ) * data[0] +
                xx * ( 1-yy ) * data[1] +
                ( 1-xx ) * yy * data[ image_->step ] +  // I(i+1,j) //这里相当于像素的周期是image_->step，即每一行存储像素的个数为image_->step
                xx * yy * data[image_->step+1]          // I(i+1,j+1)        //data[image_->step]是I(i,j)对应的下一行像素为I(i+1,j)
        );
    }

};



int main(int argc, char* *argv)
{
    if (2 != argc)
    {
        cout << "usage: direct_sparse path_to_dataset" << endl;
        return 1;
    }

    srand( (unsigned int) time(0) );        // 以时间作为随机点种子
    string path_to_dataset = argv[1];
    string associate_file = path_to_dataset + "/associate.txt";

    ifstream fin (associate_file);

    string rgb_file, depth_file, time_rgb, time_depth;
    cv::Mat color, depth, gray;             // 定义3种图像
    vector<Measurement> measurements;       // 定义测量值

    // 相机内参
    float cx = 325.5;
    float cy = 253.5;
    float fx = 518.0;
    float fy = 519.0;
    float depth_scale = 1000.0;
    Eigen::Matrix3f K;
    K << fx, 0.f, cx, 0.f, fy, cy, 0.f, 0.f, 1.0f;

    Eigen::Isometry3d Tcw = Eigen::Isometry3d::Identity();      // 定义变换矩阵

    cv::Mat prev_color;         // 只用来记录第一张图片

    // 我们以第一个图像为参考，对后续图像和参考图像做直接法 ，每一副图像 都会与第一帧图像做直接法计算第一帧到当前帧的RT 但是经过更多的帧后 关键点的数量会减少，
    // 所以实际应用时 应当规定关键点的数量少于多少 就该从新设定参考系，再次利用直接法 ，但是会累计的误差需要解决？？？？
    for (int index=0; index<9; index ++)
    {
        cout << "*********** loop " << index << " *****begin******" << endl;
        fin >> time_rgb >> rgb_file >> time_depth >> depth_file;
        color = cv::imread(path_to_dataset + "/" + rgb_file);
        depth = cv::imread(path_to_dataset + "/" + depth_file, -1);
        // flags = -1：imread按解码得到的方式读入图像
        // flags = 0：imread按单通道的方式读入图像，即灰白图像
        // flags = 1：imread按三通道方式读入图像，即彩色图像
        if ( nullptr==color.data || nullptr==depth.data )
            continue;

        cv::cvtColor( color, gray, cv::COLOR_BGR2GRAY );        // 将彩色图转换为灰度图

        if ( 0 == index )
        {
            // 对第一帧提取FAST特征点，直接法的思路是根据当前相机的位姿估计值来寻找p2的位置，我们通过优化光度误差来优化位姿，寻找与p1更为相似的p2
            vector<cv::KeyPoint> keypoints;
            cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create();
            detector->detect( color, keypoints );

            for ( auto kp:keypoints )
            {
#if SEMIDENSE
                // 表示利用半稠密法的思想，只不过结果由原来的1402个点 变为了201个点 特征点数目降低了 但是看起来精度还是很高 可以适当调整梯度域值
                Eigen::Vector2d delta(      // 计算像素坐标系下，两个方向的变化量
                    gray.ptr<uchar>(int ( kp.pt.y))[ int(kp.pt.x+1)] - gray.ptr<uchar>(int(kp.pt.y))[int(kp.pt.x-1)],
                    gray.ptr<uchar>(int(kp.pt.y+1))[int(kp.pt.x)] - gray.ptr<uchar>(int(kp.pt.y-1))[int(kp.pt.x)]
                );
                // cout <<" keypoints坐标值: "<<kp.pt.x<<" "<<kp.pt.y<<endl;//可以看出点虽然存储方式是浮点数 但是实际的值都是int类型
                if ( delta.norm() < GradientThread )
                    continue;
#endif
                // 去掉邻近边缘处的点
                if ( kp.pt.x < 20 || kp.pt.y < 20 || ( kp.pt.x+20 ) >color.cols || ( kp.pt.y+20 ) >color.rows )
                    continue;
                ushort d = depth.ptr<ushort> (cvRound(kp.pt.y)) [ cvRound(kp.pt.x) ];       // 找到深度图中对应点的深度值
                if ( 0 == d )
                    continue;

                Eigen::Vector3d p3d = project2Dto3D( kp.pt.x, kp.pt.y, d, fx, fy, cx, cy, depth_scale );    // 像素坐标系转相机坐标系

#if BookExercise // 计算小块平均灰度值作为单一像素的测量值，增加算法健壮性
                float grayscale = getPatchAverageGray(gray, kp.pt.x, kp.pt.y, PATCH_RADIUS_ONE);
#else
                float grayscale = float ( gray.ptr<uchar> ( cvRound(kp.pt.y) ) [ cvRound(kp.pt.x) ] );
#endif
                measurements.push_back( Measurement ( p3d, grayscale ) );
            }
            prev_color = color.clone();     // 只用来记录第一张照片
            continue;
        }

        // 使用直接法计算相机运动，从第二帧开始计算相机位姿g2o优化，不再提取关键点
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

        poseEstimationDirect( measurements, &gray, K, Tcw );                // 重中之重，最重要的一个函数，

        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> (t2-t1);
        cout << "direct method costs time: " << time_used.count() << " seconds." << endl;
        cout << "Tcw = " << endl << Tcw.matrix() << endl;


        // plot the feature points
        cv::Mat img_show (color.rows*2, color.cols, CV_8UC3);       // 目的是为了之后对比前后两帧图像的关键点数量 所以建立一个可以存储pre_color 和color 大小的矩阵
        // 拼接两幅图片
        prev_color.copyTo( img_show(cv::Rect(0, 0, color.cols, color.rows)) );          // 第一张照片拷贝到拼接图img_show的上方
        color.copyTo ( img_show(cv::Rect(0, color.rows, color.cols, color.rows)) );         // 移动的剩下9张图拼接到图img_show的下方
        // Rect(参数)表示坐标0,0 到cols,rows 那么大的矩形
        for ( Measurement m:measurements )
        {
            if ( rand() > RAND_MAX/5 )
                continue;
            Eigen::Vector3d p = m.pos_world;            // 3D相机坐标系（第一帧 默认也是世界帧）
            Eigen::Vector2d pixel_prev = project3Dto2D ( p(0,0), p(1,0), p(2,0), fx, fy, cx, cy );

            Eigen::Vector3d p2 = Tcw * m.pos_world;     // 当前帧坐标系
            Eigen::Vector2d pixel_now  = project3Dto2D ( p2(0,0), p2(1,0), p2(2,0), fx, fy, cx, cy );
            // 对于超出当前帧图像像素坐标轴范围的点 舍弃不画
            // 前两项判断是横坐标（X）的u判断，后两项是纵坐标（Y）的判断
            if ( pixel_now(0,0) < 0 || pixel_now(0,0)>=color.cols || pixel_now(1,0)<0 || pixel_now(1,0)>=color.rows )
                continue;

            // 随机获取bgr颜色，在cv::circle中 关键点用不同颜色画
            float b = 255 * float ( rand() ) / RAND_MAX;
            float g = 255 * float ( rand() ) / RAND_MAX;
            float r = 255 * float ( rand() ) / RAND_MAX;

            // 在img_show包含两帧图像上 以关键点为圆心画圆 半径为8个像素 颜色为bgr随机组合  2表示外轮廓线宽度为2 如果为负数则表示填充圆
            // pixel_prev 都是世界坐标系下的坐标 (以第一帧为参考系) 和当前帧下的对比 可以看出关键点的数量会逐渐减少
            cv::circle ( img_show, cv::Point2d ( pixel_prev ( 0,0 ), pixel_prev ( 1,0 ) ), 8, cv::Scalar( b,g,r ), 2 );
            cv::circle ( img_show, cv::Point2d ( pixel_now ( 0,0 ), pixel_now ( 1,0 ) +color.rows ), 8, cv::Scalar( b,g,r ), 2 );//注意这里+color.rows 当前帧在img_show的下半部分
            // 连接前后两帧匹配好的点
            cv::line( img_show, cv::Point2d (pixel_prev(0,0), pixel_prev(1,0)), cv::Point2d (pixel_now(0,0), pixel_now(1,0) + color.rows), cv::Scalar(b, g, r), 1 );
        }

        cv::imshow("result", img_show);
        cv::waitKey( 0 );
        cout << "*********** loop " << index << " *****end******" << endl;
    }


    return 0;
}


bool poseEstimationDirect ( const vector< Measurement >& measurements, cv::Mat* gray, Eigen::Matrix3f& K, Eigen::Isometry3d& Tcw )
{
    // 初始化g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,1>> DirectBlock;      // 求解的向量是6＊1的， 优化的是相机位姿
    // 第一步：设置线性方程求解器
    DirectBlock::LinearSolverType* linearSolver = new g2o::LinearSolverDense< DirectBlock::PoseMatrixType > ();
    // 第二步：选定块求解器 BlockSolver，选择求解J或者H的方法
    // DirectBlock* solver_ptr = new DirectBlock ( linearSolver );
    DirectBlock* solver_ptr = new DirectBlock (  std::unique_ptr<DirectBlock::LinearSolverType>(linearSolver) );
    // 第三步：选择迭代算法，是GN、LM还是DogLeg
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( std::unique_ptr<DirectBlock>(solver_ptr) );
    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr ); // G-N
    // g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );       // L-M  首先确定损失函数F(x+△x)使用哪种方法进行近似，或者说选定哪种非线性方程最优化方法

    // 第四步：创建稀疏优化器
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );
    optimizer.setVerbose( false );

    // 第五步：设置顶点和边
    // 设置第0个顶点
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    pose->setEstimate ( g2o::SE3Quat ( Tcw.rotation(), Tcw.translation() ) );   // 设置Tcw变换位姿初值
    pose->setId ( 0 );
    optimizer.addVertex ( pose );

    // 添加边
    int id=1;
    for ( Measurement m: measurements )
    {
        EdgeSE3ProjectDirect* edge = new EdgeSE3ProjectDirect (
                m.pos_world,
                K ( 0,0 ), K ( 1,1 ), K ( 0,2 ), K ( 1,2 ), gray
        );
        edge->setVertex ( 0, pose );
        edge->setMeasurement ( m.grayscale );
        edge->setInformation ( Eigen::Matrix<double, 1, 1>::Identity() );   // 信息矩阵维度是1，由残差维度决定，残差维度是一维的灰度Ip
        edge->setId ( id ++ );
        optimizer.addEdge ( edge );
    }
    cout<<"edges in graph: "<<optimizer.edges().size() <<endl;


    // 第六步：开始优化
    optimizer.initializeOptimization();
    optimizer.optimize ( 30 );
    Tcw = pose->estimate();
}




#if BookExercise
// 获取小块平均灰度值
// gray：灰度矩阵，x，y表示以(x,y)为中心，计算的小块的平均灰度 patchsize表示块的半径
float getPatchAverageGray(const cv::Mat& gray, float u, float v, int patchRadius)
{
    int x = cvRound(u);
    int y = cvRound(v);

    if ( (patchRadius < 0) || ( (2*patchRadius+1) > 5) )
    {
        std::cout << "Error: 请修改参数PATCH_RADIUS为指定值1 or 2! " << std::endl;
        exit(1);
    }
    float grayscale = 0.0;

    for (int j=y-patchRadius; j<=y+patchRadius; j ++)
    {
        for (int i=x-patchRadius; i<=x+patchRadius; i ++)
        {
            grayscale += float ( gray.ptr<uchar>(j)[i] );
        }
    }
    grayscale /= ( (2*patchRadius+1)*(2*patchRadius+1) );

    return grayscale;
}

#endif
