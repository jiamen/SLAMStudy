/****************************
 * 题目：用直接法Bundle Adjustment 估计相机位姿，已经给定3张图，poses.txt,points.txt文件。
 *
 * 本程序学习目标：
 * 熟悉g2o库顶点和边的定义。
 * 
 * 本代码框架来自高翔，有改动
 * 公众号：计算机视觉life。发布于公众号旗下知识星球：从零开始学习SLAM
 * 时间：2019.03
****************************/

/*
#include <iostream>

using namespace std;

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <Eigen/Core>
#include <sophus/se3.h>
#include <opencv2/opencv.hpp>

#include <pangolin/pangolin.h>
#include <boost/format.hpp>
using namespace Sophus;
using namespace pangolin;
using namespace g2o;

#define DEBUG false

typedef vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> VecSE3;
typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVec3d;

// global variables
string pose_file = "../poses.txt";
string points_file = "../points.txt";

// intrinsics
float fx = 277.34;
float fy = 291.402;
float cx = 312.234;
float cy = 239.777;

inline bool bInImage(float u, float v, int w, int h)
{
    if(u>=0 && u<w && v>=0 && v<h)
        return true;
    else
        return false;
}

// bilinear interpolation
inline float GetPixelValue(const cv::Mat &img, float x, float y)
{
    uchar* data = &img.data[int(y) * img.step + int(x)];
    float xx = x - floor(x);
    float yy = y - floor(y);
    return float(
            (1 - xx) * (1 - yy) * data[0] +
            xx * (1 - yy) * data[1] +
            (1 - xx) * yy * data[img.step] +
            xx * yy * data[img.step + 1]
    );
}


// 自定义的顶点 Vertex
// g2o vertex that use sophus::SE3 as pose
class VertexSophus : public g2o::BaseVertex<6, Sophus::SE3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexSophus() {}

    ~VertexSophus() {}

    bool read(std::istream &is) {}

    bool write(std::ostream &os) const {}

    virtual void setToOriginImpl() {
        _estimate = Sophus::SE3();
    }

    // 顶点更新函数  重载
    virtual void oplusImpl(const double *update_) {
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> update(update_);
        setEstimate(Sophus::SE3::exp(update) * estimate());
    }
};


// 自定义的边 Edge
// TODO edge of projection error, implement it
// 16x1 error, which is the errors in patch
typedef Eigen::Matrix<double,16,1> Vector16d;
class EdgeDirectProjection : public g2o::BaseBinaryEdge<16, Vector16d, g2o::VertexSBAPointXYZ, VertexSophus> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeDirectProjection(float* color, cv::Mat& target)
    {
        this->origColor = color;
        this->targetImg = target;
        this->w = targetImg.cols;
        this->h = targetImg.rows;
    }

    ~EdgeDirectProjection() {}

    // 计算残差 f(x)
    virtual void computeError() override
    {
        // ----------------  开始你的代码 ----------------------//
		// 参考十四讲中直接法BA部分

		// 1.根据当前顶点估计，计算相机坐标系下坐标
		const VertexSBAPointXYZ* p = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
		const VertexSophus* v = static_cast<const VertexSophus*>(_vertices[1]);
		Eigen::Vector3d x_local = v->estimate() * p->estimate();

		// 2. 计算像素坐标
		float x = x_local[0] * fx / x_local[2] + cx;
		float y = x_local[1] * fy / x_local[2] + cy;

		// 3. 检查是否在图像内（取pitch大小为4×4）
        if(!bInImage(x-2, y-2, w, h) || !bInImage(x+1, y+1, w, h))
        {
            for(int n=0;n<16;n++)
                _error[n] = 0;
            this->setLevel(1);
        }
        else
        {
            for(int i = -2; i < 2; i++)
                for(int j = -2; j < 2; j++)
                {
                    int num = 4*i + j + 10;
                    _error(num) = origColor[num] - GetPixelValue(targetImg, x+i, y+j);
                }
        }

        // ----------------  结束你的代码 ----------------------//
    }
	
	// 下面函数不用自己写了，但是要能看懂，计算雅可比矩阵J(x)
    virtual void linearizeOplus() override
    {
        if(level()==1)
        {
            _jacobianOplusXi = Matrix<double,16,3>::Zero();
            _jacobianOplusXj = Matrix<double,16,6>::Zero();
            return;
        }
        const VertexSBAPointXYZ* vertexPw = static_cast<const VertexSBAPointXYZ* >(vertex(0));
        const VertexSophus* vertexTcw = static_cast<const VertexSophus* >(vertex(1));

        // 求相机坐标系下坐标
        Vector3d Pc = vertexTcw->estimate() * vertexPw->estimate();
        float x = Pc[0];
        float y = Pc[1];
        float z = Pc[2];
        float inv_z = 1.0/z;
        float inv_z2 = inv_z * inv_z;
        float u = x * inv_z * fx + cx;      // 归一化点
        float v = y * inv_z * fy + cy;

        // 《SLAM 14讲》 P164页 式（7.42）  投影方程关于相机坐标系下三维点的偏导数
        Matrix<double,2,3> J_Puv_Pc;
        J_Puv_Pc(0,0) = fx * inv_z;
        J_Puv_Pc(0,1) = 0;
        J_Puv_Pc(0,2) = -fx * x * inv_z2;
        J_Puv_Pc(1,0) = 0;
        J_Puv_Pc(1,1) = fy * inv_z;
        J_Puv_Pc(1,2) = -fy * y * inv_z2;

        // 《SLAM 14讲》 P164页 式（7.44）  相机坐标系下三维点关于位姿李代数的偏导数
        Matrix<double,3,6> J_Pc_kesi = Matrix<double,3,6>::Zero();
        J_Pc_kesi(0,0) = 1;
        J_Pc_kesi(0,4) = z;
        J_Pc_kesi(0,5) = -y;
        J_Pc_kesi(1,1) = 1;
        J_Pc_kesi(1,3) = -z;
        J_Pc_kesi(1,5) = x;
        J_Pc_kesi(2,2) = 1;
        J_Pc_kesi(2,3) = y;
        J_Pc_kesi(2,4) = -x;

        // 像素梯度
        Matrix<double,1,2> J_I_Puv;
        for(int i = -2; i<2; i++)
            for(int j = -2; j<2; j++) {
                int num = 4 * i + j + 10;
                J_I_Puv(0,0) = (GetPixelValue(targetImg,u+i+1,v+j) - GetPixelValue(targetImg,u+i-1,v+j))/2;
                J_I_Puv(0,1) = (GetPixelValue(targetImg,u+i,v+j+1) - GetPixelValue(targetImg,u+i,v+j-1))/2;

                // 误差函数关于三维空间点的Jacobian    《SLAM 14讲》 P165页 式（7.47）
                _jacobianOplusXi.block<1,3>(num,0) = -J_I_Puv * J_Puv_Pc * vertexTcw->estimate().rotation_matrix();
                // 误差函数关于位姿李代数的Jacobian    《SLAM 14讲》 P164页 式（7.45）
                _jacobianOplusXj.block<1,6>(num,0) = -J_I_Puv * J_Puv_Pc * J_Pc_kesi;
            }
    }
    virtual bool read(istream &in) {}

    virtual bool write(ostream &out) const {}

private:
    cv::Mat targetImg;  // the target image
    float *origColor = nullptr;   // 16 floats, the color of this point
    int w;
    int h;
};

// plot the poses and points for you, need pangolin
void Draw(const VecSE3 &poses, const VecVec3d &points);

int main(int argc, char **argv) {

    // read poses and points
    VecSE3 poses;
    VecVec3d points;

    ifstream fin(pose_file);    // 先读取3个位姿
    while (!fin.eof()) {
        double timestamp = 0;
        fin >> timestamp;
        if (timestamp == 0) break;
        double data[7];
        for (auto& d : data) fin >> d;
        poses.push_back(Sophus::SE3(
                Eigen::Quaterniond(data[6], data[3], data[4], data[5]),
                Eigen::Vector3d(data[0], data[1], data[2])
        ));
        if (!fin.good()) break;
    }
    fin.close();

    vector<float *> color;      // 注意这种变长的vector怎么存，每个vector元素不固定大小就存指针
    fin.open(points_file);      // 在读取所有观测到的点
    while (!fin.eof())
    {
        double xyz[3] = {0};
        for (int i = 0; i < 3; i++) fin >> xyz[i];
        if (xyz[0] == 0) break;
        points.push_back(Eigen::Vector3d(xyz[0], xyz[1], xyz[2]));
        float *c = new float[16];       // 指针大小在这里指定
        for (int i = 0; i < 16; i++) fin >> c[i];
        color.push_back(c);

        if (fin.good() == false) break;
    }
    fin.close();

    cout << "poses: " << poses.size() << ", points: " << points.size() << ", color: "<<color.size()<<endl;

    // read images
    vector<cv::Mat> images;
    boost::format fmt("../%d.png");
    for (int i = 0; i < 3; i++) {
        images.push_back(cv::imread((fmt % i).str(), 0));
    }
    cout<<"images: "<<images.size()<<endl;

    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // pose 维度为 6, landmark 维度为 3
    // 第1步：创建一个线性求解器LinearSolver
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    // 第2步：创建BlockSolver。并用上面定义的线性求解器初始化
    // Block* solver_ptr = new Block (  std::unique_ptr<Block::LinearSolverType>(linearSolver) );
    Block* solver_ptr = new Block (  linearSolver );
    // 第3步：创建总求解器solver。并从GN, LM, DogLeg 中选一个，再用上述块求解器BlockSolver初始化
    // g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( std::unique_ptr<Block>(solver_ptr) );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
    // 第4步：创建稀疏优化器
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );

//    // old g2o version
//    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // pose 维度为 6, landmark 维度为 3
//    Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>(); // 线性方程求解器
//    Block* solver_ptr = new Block ( linearSolver );     // 矩阵块求解器
//    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
//    g2o::SparseOptimizer optimizer;
//    optimizer.setAlgorithm ( solver );

    // 第5步：添加顶点和边
    // ----------------  开始你的代码 ----------------------//
	// 参考十四讲中直接法BA部分

	// 1. 每一个空间特征点是一个顶点
	for (int i=0; i<points.size(); i ++)
    {
	    VertexSBAPointXYZ* vertexPw = new VertexSBAPointXYZ();
	    vertexPw->setEstimate(points[i]);
	    vertexPw->setId(i);
	    vertexPw->setMarginalized(true);
	    optimizer.addVertex(vertexPw);
    }

	// 2. 每一个位姿是一个顶点
	for (int j=0; j<poses.size(); j ++)
    {
	    VertexSophus* vertexTcw = new VertexSophus();       // 这里使用自定义的顶点
	    vertexTcw->setEstimate(poses[j]);
	    vertexTcw->setId(j + points.size());        // 注意这里的序号累计
	    optimizer.addVertex(vertexTcw);
    }

	// 3. 添加边
	for (int c=0; c<poses.size(); c ++)
    {   // 一个pose 对应多个空间点
	    for (int p=0; p<points.size(); p ++)
        {
	        EdgeDirectProjection* edge = new EdgeDirectProjection(color[p], images[c]);
	        edge->setVertex(0, dynamic_cast<VertexSBAPointXYZ*>(optimizer.vertex(p)));      // 第一个必须设置空间点Pose
	        edge->setVertex(1, dynamic_cast<VertexSophus*>(optimizer.vertex(c + points.size())));   // 第二个设置为pose
	        edge->setInformation(Matrix<double, 16, 16>::Identity());
	        RobustKernelHuber* rk = new RobustKernelHuber;      // 设置Huber鲁棒核函数
	        rk->setDelta(1.0);
	        edge->setRobustKernel(rk);
	        optimizer.addEdge(edge);
        }
    }


    // ----------------  结束你的代码 ----------------------//

    // 第6步：执行优化
    optimizer.initializeOptimization(0);
    optimizer.optimize(200);

    // 从optimizer中获取结果
    for(int c = 0; c < poses.size(); c++)
        for(int p = 0; p < points.size(); p++) {
            Vector3d Pw = dynamic_cast<VertexSBAPointXYZ*>(optimizer.vertex(p))->estimate();
            points[p] = Pw;
            SE3 Tcw = dynamic_cast<VertexSophus*>(optimizer.vertex(c + points.size()))->estimate();
            poses[c] = Tcw;
        }


    // plot the optimized points and poses
    Draw(poses, points);

    // delete color data
    for (auto &c: color) delete[] c;
    return 0;
}

// 画图函数，无需关注
void Draw(const VecSE3 &poses, const VecVec3d &points) {
    if (poses.empty() || points.empty()) {
        cerr << "parameter is empty!" << endl;
        return;
    }
    cout<<"Draw poses: "<<poses.size()<<", points: "<<points.size()<<endl;
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

        // draw poses
        float sz = 0.1;
        int width = 640, height = 480;
        for (auto &Tcw: poses) {
            glPushMatrix();
            Sophus::Matrix4f m = Tcw.inverse().matrix().cast<float>();
            glMultMatrixf((GLfloat *) m.data());
            glColor3f(1, 0, 0);
            glLineWidth(2);
            glBegin(GL_LINES);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glEnd();
            glPopMatrix();
        }

        // points
        glPointSize(2);
        glBegin(GL_POINTS);
        for (size_t i = 0; i < points.size(); i++) {
            glColor3f(0.0, points[i][2]/4, 1.0-points[i][2]/4);
            glVertex3d(points[i][0], points[i][1], points[i][2]);
        }
        glEnd();

        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}
*/

/****************************
 * 题目：用直接法Bundle Adjustment 估计相机位姿，已经给定3张图，poses.txt,points.txt文件。
 *
 * 本程序学习目标：
 * 熟悉g2o库顶点和边的定义。
 *
 * 本代码框架来自高翔，有改动
 * 公众号：计算机视觉life。发布于公众号旗下知识星球：从零开始学习SLAM
 * 时间：2019.03
****************************/

#include <iostream>

using namespace std;

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <Eigen/Core>
#include <sophus/se3.h>
#include <opencv2/opencv.hpp>

#include <pangolin/pangolin.h>
#include <boost/format.hpp>
using namespace Sophus;
using namespace pangolin;
using namespace g2o;

#define DEBUG false

typedef vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> VecSE3;
typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVec3d;

// global variables
string pose_file = "../poses.txt";
string points_file = "../points.txt";

// intrinsics
float fx = 277.34;
float fy = 291.402;
float cx = 312.234;
float cy = 239.777;

inline bool bInImage(float u, float v, int w, int h)
{
    if(u>=0 && u<w && v>=0 && v<h)
        return true;
    else
        return false;
}
// bilinear interpolation  双线性插值
inline float GetPixelValue(const cv::Mat &img, float x, float y) {
    uchar *data = &img.data[int(y) * img.step + int(x)];
    float xx = x - floor(x);
    float yy = y - floor(y);
    return float(
            (1 - xx) * (1 - yy) * data[0] +
            xx * (1 - yy) * data[1] +
            (1 - xx) * yy * data[img.step] +
            xx * yy * data[img.step + 1]
    );
}


// g2o vertex that use sophus::SE3 as pose
class VertexSophus : public g2o::BaseVertex<6, Sophus::SE3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexSophus() {}

    ~VertexSophus() {}

    bool read(std::istream &is) {}

    bool write(std::ostream &os) const {}

    virtual void setToOriginImpl() {
        _estimate = Sophus::SE3();
    }

    virtual void oplusImpl(const double *update_) {
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> update(update_);
        setEstimate(Sophus::SE3::exp(update) * estimate());
    }
};

// TODO edge of projection error, implement it
// 16x1 error, which is the errors in patch
typedef Eigen::Matrix<double, 16, 1> Vector16d;
class EdgeDirectProjection : public g2o::BaseBinaryEdge<16, Vector16d, g2o::VertexSBAPointXYZ, VertexSophus>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeDirectProjection(float *color, cv::Mat &target)
    {
        this->origColor = color;
        this->targetImg = target;
        this->w = targetImg.cols;
        this->h = targetImg.rows;
    }

    ~EdgeDirectProjection() {}

    virtual void computeError() override
    {
        // ----------------  开始你的代码 ----------------------//
        // 参考十四讲中直接法BA部分

        // 1. 跟据当前顶点估计, 计算相机坐标系下坐标
        const VertexSophus* v = static_cast<const VertexSophus*>(_vertices[1]);
        const VertexSBAPointXYZ* p = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
        Eigen::Vector3d x_local = v->estimate() * p->estimate();

        // 2. 计算像素坐标
        float x = x_local[0] * fx / x_local[2] + cx;
        float y = x_local[1] * fy / x_local[2] + cy;

        // 3. 检查是否在图像内(取 pitch 大小为 4x4)
        if(!bInImage(x-2, y-2, w, h) || !bInImage(x+1, y+1, w, h))
        {
            for(int n=0;n<16;n++)
                _error[n] = 0;
            this->setLevel(1);
        }
        else
        {
            for(int i = -2; i < 2; i++)
                for(int j = -2; j < 2; j++)
                {
                    int num = 4*i + j + 10;
                    _error(num) = origColor[num] - GetPixelValue(targetImg, x+i, y+j);
                }
        }
        // ----------------  结束你的代码 ----------------------//
    }

    // 下面函数不用自己写了，但是要能看懂
    virtual void linearizeOplus() override
    {
        if(level() == 1)
        {
            _jacobianOplusXi = Matrix<double,16,3>::Zero();
            _jacobianOplusXj = Matrix<double,16,6>::Zero();
            return;
        }

        const VertexSBAPointXYZ* vertexPw = static_cast<const VertexSBAPointXYZ* >(vertex(0));
        const VertexSophus* vertexTcw = static_cast<const VertexSophus* >(vertex(1));

        // 求相机坐标系下坐标
        Vector3d Pc = vertexTcw->estimate() * vertexPw->estimate();

        float x = Pc[0];
        float y = Pc[1];
        float z = Pc[2];
        float inv_z = 1.0/z;
        float inv_z2 = inv_z * inv_z;
        float u = x * inv_z * fx + cx;
        float v = y * inv_z * fy + cy;

        // 投影方程关于相机坐标系下三维点的偏导数
        Matrix<double,2,3> J_Puv_Pc;
        J_Puv_Pc(0,0) = fx * inv_z;
        J_Puv_Pc(0,1) = 0;
        J_Puv_Pc(0,2) = -fx * x * inv_z2;
        J_Puv_Pc(1,0) = 0;
        J_Puv_Pc(1,1) = fy * inv_z;
        J_Puv_Pc(1,2) = -fy * y * inv_z2;


        // 相机坐标系下三维点关于位姿李代数的偏导数
        Matrix<double,3,6> J_Pc_kesi = Matrix<double,3,6>::Zero();
        J_Pc_kesi(0,0) = 1;
        J_Pc_kesi(0,4) = z;
        J_Pc_kesi(0,5) = -y;
        J_Pc_kesi(1,1) = 1;
        J_Pc_kesi(1,3) = -z;
        J_Pc_kesi(1,5) = x;
        J_Pc_kesi(2,2) = 1;
        J_Pc_kesi(2,3) = y;
        J_Pc_kesi(2,4) = -x;

        // 像素梯度
        Matrix<double,1,2> J_I_Puv;
        for(int i = -2; i<2; i ++)
            for(int j = -2; j<2; j ++) {
                int num = 4 * i + j + 10;
                J_I_Puv(0,0) = (GetPixelValue(targetImg,u+i+1,v+j) - GetPixelValue(targetImg,u+i-1,v+j))/2;
                J_I_Puv(0,1) = (GetPixelValue(targetImg,u+i,v+j+1) - GetPixelValue(targetImg,u+i,v+j-1))/2;

                // 误差函数关于三维空间点的Jacobian
                _jacobianOplusXi.block<1,3>(num,0) = -J_I_Puv * J_Puv_Pc * vertexTcw->estimate().rotation_matrix();
                // 误差函数关于位姿李代数的Jacobian
                _jacobianOplusXj.block<1,6>(num,0) = -J_I_Puv * J_Puv_Pc * J_Pc_kesi;
            }
    }

    virtual bool read(istream &in) {}

    virtual bool write(ostream &out) const {}

private:
    cv::Mat targetImg;              // the target image
    float *origColor = nullptr;     // 16 floats, the color of this point
    int w;
    int h;
};

// plot the poses and points for you, need pangolin
void Draw(const VecSE3 &poses, const VecVec3d &points);

int main(int argc, char **argv)
{
    // read poses and points
    // 读取位姿
    VecSE3 poses;
    VecVec3d points;
    ifstream fin(pose_file);

    cout << "poses!!!!!!!!!!!!: " << endl;

    while (!fin.eof()) {
        double timestamp = 0;
        fin >> timestamp;
        if (timestamp == 0) break;
        double data[7];
        for (auto &d: data) fin >> d;
        poses.push_back(Sophus::SE3(
                Eigen::Quaterniond(data[6], data[3], data[4], data[5]),
                Eigen::Vector3d(data[0], data[1], data[2])
        ));
        if (!fin.good()) break;
    }
    fin.close();

    cout << "poses!!!!!!!!!!!!: " << endl;

    // 读取点
    vector<float *> color;
    fin.open(points_file);
    while (!fin.eof()) {
        // 前3个为点的空间坐标
        double xyz[3] = {0};
        for (int i = 0; i < 3; i++) fin >> xyz[i];
        if (xyz[0] == 0) break;
        // 后16个为点的
        points.push_back(Eigen::Vector3d(xyz[0], xyz[1], xyz[2]));
        float *c = new float[16];
        for (int i = 0; i < 16; i++) fin >> c[i];
        color.push_back(c);

        if (fin.good() == false) break;
    }
    fin.close();

    cout << "poses!!!!!!!!!!!!: " << endl;


    cout << "poses: " << poses.size() << ", points: " << points.size() << ", color: "<<color.size()<<endl;

    // read images
    // 读图片
    vector<cv::Mat> images;
    boost::format fmt(".././%d.png");
    for (int i = 0; i < 3; i++) {
        images.push_back(cv::imread((fmt % i).str(), 0));
    }
    cout<<"images: "<<images.size()<<endl;

    /*
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // pose 维度为 6, landmark 维度为 3
    // 第1步：创建一个线性求解器LinearSolver
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();

    // 第2步：创建BlockSolver。并用上面定义的线性求解器初始化
    Block* solver_ptr = new Block (  std::unique_ptr<Block::LinearSolverType>(linearSolver) );

    // 第3步：创建总求解器solver。并从GN, LM, DogLeg 中选一个，再用上述块求解器BlockSolver初始化
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( std::unique_ptr<Block>(solver_ptr) );

    // 第4步：创建稀疏优化器
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );
     */

    // old g2o version
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // pose 维度为 6, landmark 维度为 3
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>(); // 线性方程求解器
    Block* solver_ptr = new Block ( linearSolver );     // 矩阵块求解器
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );

    // 第5步：添加顶点和边
    // ----------------  开始你的代码 ----------------------//
    // 参考十四讲中直接法BA部分

    // 1. 每一个路标点是一个顶点
    for(int i = 0; i < points.size(); i ++) {
        VertexSBAPointXYZ* vertexPw = new VertexSBAPointXYZ();
        vertexPw->setEstimate(points[i]);
        vertexPw->setId(i);
        vertexPw->setMarginalized(true);
        optimizer.addVertex(vertexPw);
    }

    // 2. 每一个位姿是一个顶点
    for(int j = 0; j < poses.size(); j ++) {
        VertexSophus* vertexTcw = new VertexSophus();
        vertexTcw->setEstimate(poses[j]);
        vertexTcw->setId(j + points.size());
        optimizer.addVertex(vertexTcw);
    }

    // 3. 添加边
    for(int c = 0; c < poses.size(); c ++)
    {
        for(int p = 0; p < points.size(); p ++)
        {
            EdgeDirectProjection* edge = new EdgeDirectProjection(color[p],images[c]);
            edge->setVertex(0, dynamic_cast<VertexSBAPointXYZ*>(optimizer.vertex(p)));
            edge->setVertex(1,dynamic_cast<VertexSophus*>(optimizer.vertex(c + points.size())));

            // 信息矩阵维度由残差维度决定
            edge->setInformation(Matrix<double, 16, 16>::Identity());   // 这里的维度注意看目标函数，目标函数是4×4窗口,16个值的1维残差，因此这里是16

            RobustKernelHuber* rk = new RobustKernelHuber;
            rk->setDelta(1.0);
            edge->setRobustKernel(rk);
            optimizer.addEdge(edge);
        }
    }
    // ----------------  结束你的代码 ----------------------//

    // 第6步：执行优化
    optimizer.initializeOptimization(0);
    optimizer.optimize(200);

    // 从optimizer中获取结果
    for(int c = 0; c < poses.size(); c ++)
    {
        for(int p = 0; p < points.size(); p ++)
        {
            Vector3d Pw = dynamic_cast<VertexSBAPointXYZ*>(optimizer.vertex(p))->estimate();    // 获得空间点优化点
            points[p] = Pw;
        }
        SE3 Tcw = dynamic_cast<VertexSophus*>(optimizer.vertex(c + points.size()))->estimate(); // 获得位姿优化点
        poses[c] = Tcw;
    }

    for (int c = 0; c<poses.size(); c ++)
    {
        cout << "poses[" << c << "]:" << endl << poses[c] << endl;
    }

    // plot the optimized points and poses
    Draw(poses, points);

    // delete color data
    for (auto &c: color) delete[] c;
    return 0;
}

// 画图函数，无需关注
void Draw(const VecSE3 &poses, const VecVec3d &points) {
    if (poses.empty() || points.empty()) {
        cerr << "parameter is empty!" << endl;
        return;
    }
    cout<<"Draw poses: "<<poses.size()<<", points: "<<points.size()<<endl;
    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

        // draw poses
        float sz = 0.1;
        int width = 640, height = 480;
        for (auto &Tcw: poses) {
            glPushMatrix();
            Sophus::Matrix4f m = Tcw.inverse().matrix().cast<float>();
            glMultMatrixf((GLfloat *) m.data());
            glColor3f(1, 0, 0);
            glLineWidth(2);
            glBegin(GL_LINES);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glEnd();
            glPopMatrix();
        }

        // points
        glPointSize(2);
        glBegin(GL_POINTS);
        for (size_t i = 0; i < points.size(); i++) {
            glColor3f(0.0, points[i][2]/4, 1.0-points[i][2]/4);
            glVertex3d(points[i][0], points[i][1], points[i][2]);
        }
        glEnd();

        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}

