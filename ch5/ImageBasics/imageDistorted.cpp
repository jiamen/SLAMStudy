//
// Created by zlc on 2021/5/6.
// Description: 将畸变的图形进行去畸变操作，并显示去畸变和未去畸变的图像

#include <iostream>
#include <opencv2/opencv.hpp>

// 首先确定5个畸变参数k1,k2,k3,p1,p2，其中k1,k2,k3纠正径向畸变，p1,p2纠正切向畸变
/*
 * r = sqrt(x*x + y*y)
 * x_d = x(1 + k1r^2 + k2r^4 + k3r^6 ) + 2p1xy + p2(r^2 + 2x^2)
 * y_d = y(1 + k1r^2 + k2r^4 + k3r^6 ) + 2p2xy + p1(r^2 + 2y^2)
 * */
// 最后将畸变纠正后的归一化下的x_d、y_d变为像素坐标即可
/*
 * u_d = fx*x_d + cx
 * v_d = fy*y_d + cy
 * */

#include <string>

using namespace std;

string image_file = "../test.png";      // 图像路径

int main(int argc, char* *argv)
{
    // 畸变参数
    double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359, p2 = 1.76187114e-05;
    // 内参K参数
    double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;

    cv::Mat image = cv::imread(image_file, 0);          // 图像是灰度图，CV_8UC1
    // flag=-1时，8位深度，原通道
    // flag=0，8位深度，1通道
    // flag=1,   8位深度  ，3通道
    // flag=2，原深度，1通道
    // flag=3,  原深度，3通道
    // flag=4，8位深度 ，3通道
    int rows = image.rows, cols = image.cols;
    cv::Mat image_undistort = cv::Mat(rows, cols, CV_8UC1);     // 定义去畸变之后的图像

    // 遍历每个像素，把每个像素进行去畸变,注意，去畸变纠正的是像素的位置，并不是像素值本身
    for (int v=0; v<rows; v ++)
    {
        for (int u=0; u<cols; u ++)
        {
            // 先由现在的图像像素，得到归一化坐标系下的像素
            double x = (u-cx)/fx, y = (v-cy)/fy;
            double r = sqrt(x*x + y*y);
            double x_distorted = x*(1 + k1*r*r + k2*r*r*r*r ) + 2*p1*x*y + p2*(r*r+2*x*x);
            double y_distorted = y*(1 + k1*r*r + k2*r*r*r*r ) + 2*p2*x*y + p1*(r*r+2*y*y);
            double u_distorted = fx*x_distorted + cx;
            double v_distorted = fy*y_distorted + cy;

            // 赋值(最近邻插值)
            if (u_distorted >= 0 && v_distorted >= 0 && u_distorted < cols && v_distorted < rows)
            {
                image_undistort.at<uchar>(v, u) = image.at<uchar>((int)v_distorted, (int)u_distorted);
            }
            else
            {
                image_undistort.at<uchar>(v, u) = 0;
            }
        }
    }

    // 画出去畸变后图像
    cv::imshow("image undistorted", image_undistort);
    cv::imshow("image distorted", image);               // 畸变图像
    cv::waitKey(0);

    return 0;
}


