//
// Created by zlc on 2021/5/7.
// Description:

#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <chrono>

using namespace std;
using namespace Eigen;


int main(int argc, char* *argv)
{
    double ar=1.0, br=2.0, cr=1.0;      // 真实的参数
    double ae=2.0, be=-1.0, ce=5.0;     // 估计参数值，对这组数据进行优化
    int N = 100;                        // 数据的个数
    double w_sigma = 1.0;               // 噪声sigma的值
    double inv_sigma = 1.0 / w_sigma;

    cv::RNG rng;                        // OpenCV随机数产生器

    // 产生数据存下来
    vector<double> x_data, y_data;
    for (int i = 0; i < N; i ++)
    {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(ar*x*x + br*x + cr) + rng.gaussian(w_sigma));
    }

    // 高斯牛顿迭代
    int iters = 100;                // 迭代次数
    double cost=0, lastcost=0;      // 本次迭代和上次迭代，最终的目标函数，要将误差进行平方
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

    for (int iter=0; iter<iters; iter ++)
    {
        Matrix3d H = Matrix3d::Zero();      // 注意为什么一上来就直接定义了3×3矩阵
        Vector3d b = Vector3d::Zero();
        cost = 0;

        for (int i=0; i<N; i ++)
        {
            double xi = x_data[i], yi=y_data[i];
            double error = yi - exp(ae*xi*xi + be*xi + ce);     // 使用待估计参数计算估计值，与真值计算得到差值

            Eigen::Vector3d J;      // 雅克比矩阵
            J[0] = -xi*xi * exp(ae*xi*xi + be*xi + ce);         // 求导：de/da
            J[1] = -xi * exp(ae*xi*xi + be*xi + ce);            // de/db
            J[2] = -exp(ae*xi*xi + be*xi + ce);                 // de/dc

            H += inv_sigma * inv_sigma * J * J.transpose();
            b += -inv_sigma * inv_sigma * error * J;
            cost += error * error;
        }

        // 求解线性方程组 Hx = b
        // Vector3d dx = H.ldlt().solve(b);        // 利用ldlt分解
        // 也可以用QR分解
        Vector3d dx = H.colPivHouseholderQr().solve(b);  //利用Qr分解

        // isnan() 判断是不是NAN值（not a number非法数字）
        //　标准库中定义了一个宏：NAN来表示非法数字。
        //　比如负数开方、负数求对数、0.0/0.0、0.0* INFINITY(无穷大)、INFINITY/INFINITY、INFINITY-INFINITY
        //　以上操作都会得到NAN。
        //　注意：如果是整数0/0会产生操作异常
        // isinf()测试某个浮点数是否是无限大，其中INF表示无穷大
        if (isnan(dx[0]))
        {
            cout << "result is nan" << endl;
            break;
        }

        // 如果损失不下降，优化结束，就应该退出迭代
        if ((iter > 0) && (cost >= lastcost))
        {
            cout << "cost = " << cost << " >= " << "lastcost = " << lastcost << ", break!" << endl;
            break;
        }

        // 否则继续优化
        ae += dx[0];
        be += dx[1];
        ce += dx[2];
        lastcost = cost;

        cout << "iter: " << iter
             << ",\t\t cost: " << cost << ",\t\t update: " << dx.transpose()
             << "\t\t estimated params: " << ae << ", " << be << ", " << ce << endl;
    }

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> (t2-t1);
    cout << "cost time: " << time_used.count() * 1000 << endl;          // ms

    cout << "estimated abc = " << ae << ", " << be << ", " << ce << endl;

    return 0;
}



