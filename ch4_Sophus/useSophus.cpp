//
// Created by zlc on 2021/4/4.
//

#include <iostream>
#include <cmath>

using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sophus/so3.h"
#include "sophus/se3.h"



int main(int argc, char* *argv)
{
    // 沿Z轴转90°的旋转矩阵，旋转向量转为旋转矩阵
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0,0,1)).toRotationMatrix();

    Sophus::SO3 SO3_R(R);       // Sophus::SO(3)可以直接从旋转矩阵构造
    Sophus::SO3 SO3_v(0, 0, M_PI/2);        // Sophus::SO(3)亦可从旋转向量构造

    Eigen::Quaterniond q(R);    // 或者四元数，用旋转矩阵初始化，获得对应的四元数
    Sophus::SO3 SO3_q(q);       // 再用四元数来初始化对应的李群

    // 上述表达方式等价的
    // 输出SO(3)时，以so(3)形式输出
    cout << "SO(3) from matrix: " << SO3_R << endl;         // 0   0   1.5708
    cout << "SO(3) from vector: " << SO3_v << endl;         // 0   0   1.5708
    cout << "SO(3) from quaternion: " << SO3_q << endl;     // 0   0   1.5708

    // 使用对数映射获得它李群（旋转矩阵R）的李代数
    Eigen::Vector3d so3 = SO3_R.log();
    cout << "so3 = " << so3.transpose() << endl;            // 本来是竖向打印，现在用转置后横向输出
    // hat 为向量 到 反对称矩阵
    cout << "so3 hat = \n" << Sophus::SO3::hat(so3) << endl;
    // 相对的，vee为反对称矩阵 到 向量
    cout << "so3 hat vee = " << Sophus::SO3::vee( Sophus::SO3::hat(so3) ).transpose() << endl;       // 旋转矩阵转换为向量，向量再转置横向输出
    // so3 hat vee =   0      0 1.5708


    // 增量扰动模型的更新，P75页 △R对应的李代数为φ， △R·R = exp(φ^)·exp(Φ^)，注意φ≠Φ
    Eigen::Vector3d update_so3(1e-4, 0, 0);     // 假设更新量为这么多
    Sophus::SO3 SO3_updated = Sophus::SO3::exp(update_so3) * SO3_R;
    cout << "SO3 updated = " << SO3_updated << endl;
    // SO3 updated =  7.85398e-05 -7.85398e-05       1.5708


    cout << "*******************我是分割线*******************" << endl;
    // 对SE3(3)操作大同小异
    Eigen::Vector3d t(1,0,0);       // 沿x轴平移1
    Sophus::SE3 SE3_Rt(R, t);                // 从R，t构造SE3(3)
    Sophus::SE3 SE3_qt(q, t);                // 从q，t构造SE(3)
    cout << "SE3 from R, t = " << endl << SE3_Rt << endl;
    cout << "SE3 from q, t = " << endl << SE3_qt << endl;
    // SE3 from R, t =
    //      0   0   1.5708
    //      1   0      0

    // 李代数se(3)是一个六维向量，方便起见先typedef一下
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d se3 = SE3_Rt.log();
    cout << "se3 = " << se3.transpose() << endl;    // 观察输出，会发现在Sophus中，se(3)的平移在前，旋转在后
    // 0.785398 -0.785398         0         0         0    1.5708

    // 同样的，有hat和vee两个算符
    cout << "se3 hat = " << endl << Sophus::SE3::hat(se3) << endl;
    // se3 hat =
    //        0   -1.5708         0  0.785398
    //   1.5708         0        -0 -0.785398
    //       -0         0         0         0
    //        0         0         0         0
    cout << "se3 hat vee = " << endl << Sophus::SE3::vee( Sophus::SE3::hat(se3) ).transpose() << endl;
    // se3 hat vee = 0.785398 -0.785398         0         0         0    1.5708


    // 最后，演示一下更新
    Vector6d update_se3;    // 更新量
    update_se3.setZero();
    update_se3(0,0) = 1e-4d;
    Sophus::SE3 SE3_updated = Sophus::SE3::exp(update_se3) * SE3_Rt;
    cout << "SE3 updated = " << endl << SE3_updated.matrix() << endl;
    // SE3 updated =
    //2.22045e-16          -1           0      1.0001
    //          1 2.22045e-16           0           0
    //          0           0           1           0
    //          0           0           0           1


    return 0;
}

