//
// Created by zlc on 2021/5/6.
// Description: stereo双目点云成像（重点是获取视差，得到深度信息）
// 利用pangolin绘制点云

#include <iostream>
#include <opencv2/opencv.hpp>
// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <string>
#include <unistd.h>         // usleep函数所在头文件

#include <Eigen/Core>
// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

using namespace std;
using namespace Eigen;

// 文件路径，如果不对，请调整
string left_file  = "../left.png";
string right_file = "../right.png";
string disparity_file = "../disparity.png";

// 在panglin中画图，已写好，无需调整
void showPointCloud(const vector<Vector4d, Eigen::aligned_allocator<Vector4d>>& pointcloud);

int main(int argc, char* *argv)
{
    // 内参
    double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
    // base line 双目之间的基线
    double b = 0.573;

    // 读图像
    cv::Mat image_left  = cv::imread(left_file, 0);
    cv::Mat image_right = cv::imread(right_file, 0);
    // cv::Mat disparity_ori = cv::imread(disparity_file, 0);  // disparity 为CV_8U,单位为像素

    // static Ptr<StereoSGBM> create(int minDisparity = 0, int numDisparities = 16,           int blockSize = 3,  int P1 = 0, int P2 = 0, int disp12MaxDiff = 0,   int preFilterCap = 0, int uniquenessRatio = 0,  int speckleWindowSize = 0, int //speckleRange = 0,int mode = StereoSGBM::MODE_SGBM);
    // semi-global matching（SGM）是一种用于计算双目视觉中视差（disparity）的半全局匹配算法，
    // 在OpenCV中的实现为semi-global block matching（SGBM）
    // minDisparity 最小的可能的视差值
    // numDisparity 是最大视差减去最小视差
    // disp12MaxDiff是左右图视差检查所允许的最大的不同值
    // P1, P2：控制视差变化平滑性的参数。P1、P2的值越大，视差越平滑。P1是相邻像素点视差增/减 1 时的惩罚系数；P2是相邻像素点视差变化值大于1时的惩罚系数。P2必须大于P1。
    // preFilterCap：预处理滤波器的截断值，预处理的输出值仅保留[-preFilterCap, preFilterCap]范围内的值，
    // speckleWindowSize：检查视差连通区域变化度的窗口大小, 值为 0 时取消 speckle 检查，int 型
    // speckleRange：视差变化阈值，当窗口内视差变化大于阈值时，该窗口内的视差清零，int 型
    // uniquenessRatio：视差唯一性百分比， 视差窗口范围内最低代价是次低代价的(1 + uniquenessRatio/100)倍时，最低代价对应的视差值才是该像素点的视差，否则该像素点的视差为 0

    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32);
    cv::Mat disparity_sgbm, disparity;          // disparity 是视差图, d = uL-uR
    sgbm->compute(image_left, image_right, disparity_sgbm);

    // 4-byte floating point (float)
    // CV_32F是 float -像素是在0-1.0之间的任意值，这对于一些数据集的计算很有用，但是它必须通过将每个像素乘以255来转换成8位来保存或显示
    disparity_sgbm.convertTo(disparity, CV_32F, 1.0 / 16.0f);// disparity is optical parallax (shi cha)

    // 生成点云, Eigen::aligned_allocator<Vector4d> Eigen管理内存和C++11中的方法是不一样的，所以需要单独强调元素的内存分配和管理
    vector<Vector4d, Eigen::aligned_allocator<Vector4d>> pointcloud;

    // 如果你的机器慢，请把后面的v++和u++改成v+=2, u+=2
    for (int v=0; v<image_left.rows; v ++)
    {
        for (int u=0; u<image_right.cols; u ++)
        {
            // 检查计算出的视差范围
            if (disparity.at<float>(v, u) <= 0.0 || disparity.at<float>(v, u) >= 96.0)
                continue;

            // unsigned int disp = disparity_ori.ptr<unsigned short>(v)[u];
            // if (0 == disp)
            // {
            //     cout << "disp = 0" << endl;
            //     continue;
            // }

            Vector4d point(0,0,0, image_left.at<uchar>(v,u)/255.0);     // 前三维为xyz，第四维为颜色

            // 有像素(u,v)，有深度d=z，直接计算相机坐标系下的点，完全按照P102页计算
            // double depth = (fx * b * 1000) / disp;      // 计算中是以毫米为单位的
            // double x = (u-cx) / fx;
            // double y = (v-cy) / fy;
            // point[0] = x * depth;
            // point[1] = y * depth;
            // point[2] = depth;

            // 根据双目模型计算point的位置
            double x = (u-cx) / fx;     // 归一化坐标系下的x
            double y = (v-cy) / fy;     // 归一化坐标系下的y
            double depth = fx * b / (disparity.at<float>(v, u));    // P91 (5.16)
            // 转换到相机坐标系下，每个点的坐标
            point[0] = x * depth;
            point[1] = y * depth;
            point[2] = depth;

            pointcloud.push_back(point);
        }
    }

    // 画出点云
    showPointCloud(pointcloud);
    return 0;
}


void showPointCloud(const vector<Vector4d, Eigen::aligned_allocator<Vector4d>>& pointcloud)
{
    // pangolin的使用可以参考文章: https://blog.csdn.net/joun772/article/details/109246680
    // 判断点云存不存在
    if(pointcloud.empty())
    {
        cerr << "point cloud is empty" << endl;
        return;
    }
    // 设置窗口
    pangolin::CreateWindowAndBind("Point Cloud Viewer",1024,768);
    // 开启深度的测试？
    glEnable(GL_DEPTH_TEST);
    // 开启融合
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            //
            pangolin::ProjectionMatrix(1024,768,500,500,512,389,0.1,1000),
            //
            pangolin::ModelViewLookAt(0,-0.1,-1.8,0,0,0,0.0,-1.0,0.0)
    );
    pangolin::View &d_cam=pangolin::CreateDisplay()
            .SetBounds(0.0,1.0,pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        glPointSize(2);     // 点的尺寸
        glBegin(GL_POINTS);      //
        // 遍历点云
        for(auto& p : pointcloud)
        {
            glColor3f(p[3],p[3],p[3]);
            glVertex3d(p[0],p[1],p[2]);
        }
        glEnd();

        pangolin::FinishFrame();
        usleep(5000);           // 休眠 5ms
    }

    return ;
}
