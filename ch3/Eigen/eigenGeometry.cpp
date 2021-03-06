//
// Created by zlc on 2021/5/5.
//

#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

/*
 * 本程序演示了Eigen几何模块的使用方法
 * */

int main(int argc, char* *argv)
{
    // Eigen/Geometry 模块提供了各种旋转和平移的表示
    // 3D 旋转矩阵直接使用 Matrix3d 或 Matrix3f
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();

    // 旋转向量使用AngleAxis，它低层不直接是Matrix，但运算可以当作矩阵（因为重载了运算符）
    Eigen::AngleAxisd rotation_vector( M_PI/4, Eigen::Vector3d(0,0,1) );    // 沿Z轴旋转45°
    cout.precision(3);
    cout << "rotation vector's angle = " << rotation_vector.angle() << endl;                    // 旋转向量的角度（大小）
    cout << "rotation vector's angle = " << rotation_vector.axis().transpose() << endl;         // 旋转向量的方向
    cout << "rotation matrix = \n" << rotation_vector.matrix() << endl;

    // 也可以直接赋值，直接将角轴（旋转向量）转换成旋转矩阵
    rotation_matrix = rotation_vector.toRotationMatrix();

    // 用AngleAxis可以进行坐标变换
    Eigen::Vector3d v(1,0,0);
    Eigen::Vector3d v_rotated = rotation_vector * v;
    cout << "(1,0,0) after rotation = " << v_rotated.transpose() << endl;

    // 或者用旋转矩阵
    v_rotated = rotation_matrix * v;
    cout << "(1,0,0) after rotation = " << v_rotated.transpose() << endl;

    // 欧拉角：可以将旋转矩阵直接转换成欧拉角
    Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2,1,0);              // ZYX顺序，即roll pitch yaw
    cout << "yaw pitch roll = " << euler_angles.transpose() << endl;

    // 欧式变换矩阵使用Eigen::Isometry
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();        /* △△ 虽然称为3d，实质上是4×4的矩阵 △△*/
    T.rotate( rotation_vector );                                // 按照rotation_vector进行旋转
    T.pretranslate( Eigen::Vector3d(1,3,4) );   // 把平移向量设成(1,3,4)
    cout << "Transform matrix = \n" << T.matrix() << endl;

    // 用变换矩阵进行坐标变换
    Eigen::Vector3d v_transformed = T * v;                      // 相当于R*v+t
    cout << "v transformed = " << v_transformed.transpose() << endl;


    // 对于仿射和射影变换，使用Eigen::Affine3d和Eigen::Projective3d 即可，略
    // 仿射变换Affine3d 和 射影变换Projective3d
    cout << endl << "********************仿射变换Affine3d和射影变换Projective3d**************************" << endl;
    Eigen::Affine3d Aff = Eigen::Affine3d::Identity();
    Aff.rotate(rotation_vector);
    Aff.pretranslate(Eigen::Vector3d(1, 3, 4));     // 平移
    cout << "Affine matrix = " << endl << Aff.matrix() << endl;

    Eigen::Projective3d Pro = Eigen::Projective3d::Identity();
    Pro.rotate(rotation_vector);
    Pro.pretranslate(Eigen::Vector3d(1,3,4));
    cout << "Projective matrix = " << endl << Pro.matrix() << endl;


    // 四元数
    // 可以直接把AngleAxis赋值给四元数，反之亦然
    Eigen::Quaterniond q = Eigen::Quaterniond( rotation_vector );
    cout << "quaternion = \n" << q.coeffs().transpose() << endl;            // 请注意coeffs的顺序是(x,y,z,w)，w为实部，前三者为虚部

    // 也可以把旋转矩阵赋给它
    q = Eigen::Quaterniond( rotation_matrix );
    cout << "quaternion = \n" << q.coeffs().transpose() << endl;

    // 使用四元数旋转一个向量，使用重载的乘法即可
    v_rotated = q * v;                // 注意数学上是qvq^(-1)
    cout << "(1,0,0) after rotation = " << v_rotated.transpose() << endl;
    cout << "用数学方法表述 = q * quaterniond(0,1,0,0) * q(-1) = " << (q*Eigen::Quaterniond(0,1,0,0)*q.inverse()).coeffs().transpose() << endl;


    return 0;
}

