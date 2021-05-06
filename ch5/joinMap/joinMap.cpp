//
// Created by zlc on 2021/4/5.
//

#include <iostream>
#include <fstream>
#include <unistd.h>     // usleep函数

using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry>
#include <boost/format.hpp>
// pcl 相关头文件
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pangolin/pangolin.h>
#include <Eigen/Core>

using namespace Eigen;

typedef Eigen::Matrix<double,6,1> Vector6d;
void showPointCloud(const vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud);


int main(int argc, char* *argv)
{
    vector<cv::Mat> colorImgs, depthImgs;   // 彩色图和深度图
    vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;   // 相机位姿

    ifstream fin("../pose.txt");
    if (!fin)
    {
        cerr << "请在有pose.txt的目录下运行此程序" << endl;
        return 1;
    }

    for( int i=0; i<5; i ++ )
    {
        // 文件夹下的照片名字格式
        boost::format fmt("./%s/%d.%s");    // 图像文件格式
        colorImgs.push_back( cv::imread( (fmt%"../color"%(i+1)%"png").str() ) );
        depthImgs.push_back( cv::imread( (fmt%"../depth"%(i+1)%"pgm").str(), -1 ) );       // 使用-1读取原始图像

        double data[7] = {0};
        for ( auto& d : data )                  // 读取一行数据，读7个
            fin >> d;

        // 由读取到的旋转四元数和平移向量，变换为变换矩阵T_wc
        Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);
        Eigen::Isometry3d  T(q);
        T.pretranslate( Eigen::Vector3d(data[0], data[1], data[2]) );
        poses.push_back( T );                   // T_wc
    }

    // 计算点云并拼接
    // 相机内参K参数
    double cx = 325.5;
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    double depthScale = 1000.0;     // 获取深度像素对应长度单位（米）的换算比例

    cout << "正在将图像转换为点云..." << endl;

    // 定义点云使用的格式：这里用的是 XYZRGB
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    // 自添加：定义一个显示用的点云
    vector<Vector6d, Eigen::aligned_allocator<Vector6d>> pointcloud;

    // 新建一个点云
    PointCloud::Ptr pointCloud( new PointCloud );
    for ( int i=0; i<5; i ++ )
    {
        cout << "转换图像中：" << i+1 << endl;
        cv::Mat color = colorImgs[i];
        cv::Mat depth = depthImgs[i];
        Eigen::Isometry3d T = poses[i];             // 第i张照片对应的，从 相机坐标系 转换为 世界坐标系 的 变换矩阵

        for ( int v=0; v<color.rows; v ++ )         // 行，左上角为原点，向下竖着走为v
        {
            for ( int u=0; u<color.cols; u ++ )     // 列，左上角为原点，向右横着走为u
            {
                unsigned int d = depth.ptr<unsigned short> (v)[u];  // 深度值
                if ( 0 == d )
                    continue;
                // 由像素坐标系转换为
                Eigen::Vector3d point;
                point[2] = double(d)/depthScale;            // Z,获取深度像素对应长度单位（米）的换算比例
                point[0] = (u-cx) * point[2] / fx;          // X
                point[1] = (v-cy) * point[2] / fy;          // Y
                Eigen::Vector3d pointWorld = T * point;     // 世界坐标系下的点

                PointT p;
                p.x = pointWorld[0];
                p.y = pointWorld[1];
                p.z = pointWorld[2];
                p.b = color.data[ v*color.step + u*color.channels() ];      // color.step等于一行上的列数
                p.g = color.data[ v*color.step + u*color.channels()+1 ];
                p.r = color.data[ v*color.step + u*color.channels()+2 ];
                pointCloud->points.push_back( p );

                // 自添加：用来显示的
                Vector6d p_v;//前三个是坐标，后三个是颜色
                p_v.head<3>()=pointWorld;
                p_v[5]=color.data[v*color.step+u*color.channels()];//BLUE
                p_v[4]=color.data[v*color.step+u*color.channels()+1];//GREEN
                p_v[3]=color.data[v*color.step+u*color.channels()+2];//RED
                pointcloud.push_back(p_v);
            }
        }
    }

    pointCloud->is_dense = false;

    cout << "点云共有" << pointCloud->size() << "个点。" << endl;
    pcl::io::savePCDFileBinary("map.pcd", *pointCloud);         // 存为.pcd文件，使用pcl_viewer查看

    // 自添加：用来显示拼接点云
    showPointCloud(pointcloud);

    return 0;
}

// void showPointCloud(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>>& pointcloud)
void showPointCloud(const vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud)
{
    // pangolin的使用可以参考我之前的文章
    // https://blog.csdn.net/joun772/article/details/109246680
    // 判断点云存不存在
    if(pointcloud.empty())
    {
        cerr<<"point cloud is empty"<<endl;
        return;
    }
    // 设置窗口
    pangolin::CreateWindowAndBind("Point Cloud Viewer",1024,768);
    // 开启深度的测试？
    glEnable(GL_DEPTH_TEST);
    // 开启融合
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    pangolin::OpenGlRenderState s_cam(
            //
            pangolin::ProjectionMatrix(1024,768,500,500,512,389,0.1,1000),
            //
            pangolin::ModelViewLookAt(0,-0.1,-1.8,0,0,0,0.0,-1.0,0.0)
    );
    pangolin::View &d_cam=pangolin::CreateDisplay()
            .SetBounds(0.0,1.0,0, 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));
    while (pangolin::ShouldQuit()== false)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        glPointSize(2);     // 点的尺寸
        glBegin(GL_POINTS);      // 开始画
        //
        for(auto& p : pointcloud)
        {
            glColor3f(p[3]/255.0,p[4]/255.0,p[5]/255.0);
            glVertex3d(p[0],p[1],p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);   // 休眠 5ms
    }

    return;
}


