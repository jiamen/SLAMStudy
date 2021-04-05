//
// Created by zlc on 2021/4/5.
//

#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>

using namespace std;


// 代价函数的计算模型CostFunction模型，在类中定义带模板参数的()运算符，该类成为拟函数(Function),
// ceres可以向调用函数一样，对该类的某个对象（比如a）调用a<double>()方法，使对象具有函数的行为。
struct CURVE_FITTING_COST
{
    const double _x, _y;        // _x,_y数据
    CURVE_FITTING_COST( double x, double y ) : _x(x), _y(y)    {    }

    // 残差的计算
    template <typename T>
    bool operator() ( const T* const abc,       // 模型参数，有3维
                      T* residual ) const       // 残差
    {
        residual[0] = T(_y) - ceres::exp( abc[0] * T(_x) * T(_x) + abc[1]*T(_x) + abc[2] );     // y - exp(ax^2+bx+c)
        return true;
    }
};


int main(int argc, char* *argv)
{
    double a=1.0, b=2.0, c=1.0;     // 真实参数值
    int N = 100;                    // 数据点
    double w_sigma = 1.0;           // 噪声Sigma值
    cv::RNG rng;                    // OpenCV随机数产生器

    double abc[3] = {0,0,0};        // abc参数的估计值，这是估计出来的系数值

    vector<double> x_data, y_data;  // 真实参数产生的带噪声数据

    // 生成原始数据并添加噪声
    cout << "generating data: " << endl;
    for ( int i=0; i<N; i ++ )
    {
        double x = i / 100.0;
        x_data.push_back( x );
        y_data.push_back( exp(a*x*x + b*x + c) + rng.gaussian(w_sigma) );
        cout << x_data[i] << " " << y_data[i] << endl;
    }


    // 构建最小二乘问题
    ceres::Problem problem;
    for ( int i=0; i<N; i ++ )
    {
        problem.AddResidualBlock(       // 向问题中添加误差项
        // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
            new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3> (
                new CURVE_FITTING_COST ( x_data[i], y_data[i] )
            ),
            nullptr,         // 核函数，这里不使用，为空
            abc                         // 待估计参数
        );
    }

    // 配置求解器
    ceres::Solver::Options options;     // 这里有很多配置项可以填
    options.linear_solver_type = ceres::DENSE_QR;   // 增量方程如何求解
    options.minimizer_progress_to_stdout = true;    // 输出到cout

    ceres::Solver::Summary summary;     // 优化信息
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    ceres::Solve( options, &problem, &summary );    // 开始优化
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
    cout << "solve time cost = " << time_used.count() << " seconds. " << endl;


    // 输出结果
    cout << summary.BriefReport() << endl;
    cout << "estimated a, b, c = ";
    for ( auto a : abc )
        cout << a << " ";
    cout << endl;


    return 0;
}

