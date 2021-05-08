//
// Created by zlc on 2021/5/5.
//

#include <iostream>
#include <ctime>

using namespace std;

// Eigen 部分
#include <Eigen/Core>
// 稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>

#define MATRIX_SIZE 50

/*
 * 本程序演示了EIgen基本类型的使用
 * */

int main(int argc, char* *argv)
{
    // Eigen中所有向量和矩阵呢都是Eigen::Matrix, 它是一个模板类。它的前三个参数为：数据类型，行，列
    // 声明一个2*3的float矩阵
    Eigen::Matrix<float, 2, 3> matrix_23;

    // 同时，Eigen通过typedef提供了许多内置类型，不过低层仍是Eigen::Matrix
    // 例如：Vector3d实质上是Eigen::Matrix<double, 3, 1> 即三维向量
    Eigen::Vector3d v_3d;
    Eigen::Matrix<float, 3, 1> vd_3d;

    // Matrix3d 实质上是Eigen::Matrix<double,3,3>
    Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero();        // 初始化为零
    // 如果不确定矩阵大小，可以使用动态大小的矩阵
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix_dynamic;

    // 更简单的
    Eigen::MatrixXd matrix_x;
    // 这种类型还有很多

    // 下面是对Eigen阵的操作
    // 输入数据（初始化）
    matrix_23 << 1, 2, 3, 4, 5, 6;
    // 输出
    cout << matrix_23 << endl;

    // 用()访问矩阵中的元素
    for (int i=0; i<2; i ++)
    {
        for (int j=0; j<3; j ++)
        {
            cout << matrix_23(i,j) << "\t";
        }
        cout << endl;
    }

    // 矩阵和向量相乘（实际上仍是矩阵和矩阵）
    v_3d  << 3, 2, 1;
    vd_3d << 4, 5, 6;
    // 但是在Eigen里你不能混合两种不同类型的矩阵，像这样是错的
    // Eigen::Matrix<double, 2, 1> result_wrong_type = matrix_23 * v_3d;
    // 显示转换
    Eigen::Matrix<double, 2, 1> result = matrix_23.cast<double>() * v_3d;   // float转为double
    cout << "result = \n" << result << endl;

    Eigen::Matrix<float, 2, 1> result2 = matrix_23 * vd_3d;
    cout << "result2 = \n" << result2 << endl;

    // 同样你不能搞错矩阵的维度
    // 试着取消下面的注释，看看Eigen会报什么错
    // Eigen::Matrix<double, 2, 3> result_wrong_dimension = matrix_23.cast<double>() * v_3d;

    // 一些矩阵运算
    // 四则运算就不演示了，直接用+-*/即可。
    matrix_33 = Eigen::Matrix3d::Random();      // 随机数矩阵
    cout << "matrix_33 = \n" << matrix_33 << endl << endl;

    cout << "matrix_33.transpose() = \n" << matrix_33.transpose() << endl;      // 转置
    cout << "matrix_33.sum() = "   << matrix_33.sum() << endl;                  // 各元素和
    cout << "matrix_33.trace() = " << matrix_33.trace() << endl;                // 迹
    cout << "10*matrix_33 = \n" << 10*matrix_33 << endl;                        // 数乘
    cout << "matrix_33.inverse() = \n" << matrix_33.inverse() << endl;          // 逆
    cout << "matrix_33.determinant() = " << matrix_33.determinant() << endl;    // 行列式

    cout << std::endl;
    // 特征值
    // 实对称矩阵可以保证对角化成功
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver ( matrix_33.transpose()*matrix_33 );
    cout << "Eigen values = \n"  << eigen_solver.eigenvalues()  << endl;
    cout << "Eigen vectors = \n" << eigen_solver.eigenvectors() << endl;


    cout << "*********************************************" << endl;
    // 解方程
    // matrix_NN*x = v_Nd;      Ax = b, 其中A=LL^T
    // N 50
    // using 2 methods to solve linear equation
    //     1、to get inverse directly
    //     2、using QR 分解
    Eigen::Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN = Eigen::MatrixXd::Random();
    matrix_NN = matrix_NN * matrix_NN.transpose();          // 保证半正定   https://zhuanlan.zhihu.com/p/44860862
    Eigen::Matrix<double, MATRIX_SIZE, 1> v_Nd = Eigen::MatrixXd::Random();        // 随机的向量

    clock_t time_stt = clock();
    Eigen::Matrix<double, MATRIX_SIZE, 1> x = matrix_NN.inverse() * v_Nd;

    cout << "time of normal inverse is " << 1000 * (clock()-time_stt)/(double)CLOCKS_PER_SEC << "ms" << endl;
    cout << "x = " << endl << x.transpose() << endl;

    // 用cholesky 分解decomposition
    // matrix_NN * x = v_Nd
    // x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    cout << "*********************************************" << endl;
    time_stt = clock();
    x = matrix_NN.ldlt().solve(v_Nd);
    cout << "time of normal inverse is "
         << 1000 * (clock()-time_stt) / (double)CLOCKS_PER_SEC << "ms" << endl;

    cout << "x = " << endl << x.transpose() << endl;

    return 0;
}

