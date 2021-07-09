//
// Created by zlc on 2021/5/11.
//

#ifndef _VO_G2O_BAL_CLASS_H_
#define _VO_G2O_BAL_CLASS_H_

#include <Eigen/Core>
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"

#include "ceres/autodiff.h"

#include "tools/rotation.h"
#include "common/projection.h"


// 定义相机的顶点                    3维李代数表示旋转，3维表示平移,焦距f以及两个畸变系数k1，k2都需要优化，即内参外参都需要优化
class VertexCameraBAL : public g2o::BaseVertex<9, Eigen::VectorXd>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexCameraBAL() {  }

    virtual bool read (std::istream& /*is*/)
    {
        return false;
    }

    virtual bool write (std::ostream& /*os*/)  const
    {
        return false;
    }

    virtual void setToOriginImpl()  {  }

    // 更新，x_k+1 = x_k +　△x
    virtual void oplusImpl ( const double* update )
    {
        Eigen::Vector3d::ConstMapType v(update, VertexCameraBAL::Dimension);
        _estimate += v;
    }
};

// 定义空间点的顶点
class VertexPointBAL : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexPointBAL()    {  }

    virtual bool read (std::istream& /*is*/)
    {
        return false;
    }

    virtual bool write ( std::ostream& /*os*/ ) const
    {
        return false;
    }

    virtual void setToOriginImpl()  {  }

    // 更新，x_k+1 = x_k +　△x
    virtual void oplusImpl ( const double* update )
    {
        Eigen::Vector3d::ConstMapType v(update);
        _estimate += v;
    }
};


// 定义BA优化问题的边
class EdgeObservationBAL : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexCameraBAL, VertexPointBAL>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeObservationBAL() {  }

    virtual bool read (std::istream& /*is*/)
    {
        return false;
    }

    virtual bool write (std::ostream& /*is*/) const
    {
        return false;
    }

    // 计算残差：注意这里有两种写法
    virtual void computeError() override
    {
        // const VertexCameraBAL* cam  = static_cast<const VertexCameraBAL*> ( vertex(0) );
        auto cam = (VertexCameraBAL*)_vertices[0];
        const VertexPointBAL* point = static_cast<const VertexPointBAL*>  ( vertex(1) );

        // 调动下面的重载符号()进行残差计算
        ( *this ) ( cam->estimate().data(), point->estimate().data(), _error.data() );
    }

    template<typename T>
    bool operator() (const T* camera, const T* point, T* residuals) const
    {
        // 使用相机投影模型，对特征点进行操作，得到最后的像素坐标
        T predictions[2];
        CamProjectionWithDistortion( camera, point, predictions );

        // 残差 e = 预测像素坐标 - 实际像素坐标
        residuals[0] = predictions[0] - T( measurement()(0) );
        residuals[1] = predictions[1] - T( measurement()(1) );

        return true;
    }


    virtual void linearizeOplus() override
    {
        // use numeric Jacobians
        // BaseBinaryEdge<2, Vector2d, VertexCameraBAL, VertexPointBAL>::linearizeOplus();
        // return;
        // using autodiff from ceres. Otherwise, the system will use g2o numerical diff for Jacobians

        // 别忘了对顶点做强制转换
        const VertexCameraBAL* cam  = static_cast<const VertexCameraBAL*> ( vertex(0) );
        const VertexPointBAL* point = static_cast<const VertexPointBAL*>  ( vertex(1) );

        // 自动求导    提取顶点之后调用autodiff模板类，模板参数为：边，基本数据类型，N0：node0维度，N1：node1维度，将其typedef为BalAutoDiff。
        typedef ceres::internal::AutoDiff<EdgeObservationBAL, double, VertexCameraBAL::Dimension, VertexPointBAL::Dimension> BalAutoDiff;

        // 误差对相机导数2×9，误差对空间点导数2×3，定义对应存储导数的矩阵。
        Eigen::Matrix<double, Dimension, VertexCameraBAL::Dimension, Eigen::RowMajor> dError_dCamera;
        Eigen::Matrix<double, Dimension, VertexPointBAL::Dimension,  Eigen::RowMajor> dError_dPoint;

        // 定义元素为double类型的指针数组，存储相机数据(9个数)数组指针，路标点数据（3个数）数组指针
        double* parameters[] = { const_cast<double*> (cam->estimate().data()), const_cast<double*> (point->estimate().data()) };
        // 定义元素为double类型的指针数组，存储两个雅克比矩阵数据数组指针。
        double* jacobians[]  = { dError_dCamera.data(), dError_dPoint.data() };         // J = [ F E ]     后面的自动求导函数会修改这里的指针，导致dError_dCamera数据变化
        double value[Dimension];

        // 使用autodiff结构体中的静态成员函数求导函数，其中第一个变量对应模板中第一个参数就是该边,直接使用this,第二个参数为数据数组的指针数组,第三个参数为维度,value承接误差（double）,jacobians承接得到的雅克比矩阵,返回bool值
        bool diffState = BalAutoDiff::Differentiate ( *this, parameters, Dimension, value, jacobians );
        // Differentiate: 区分; 区别; 辨别; 表明…间的差别; 构成…间差别的特征; (尤指不公正地)差别对待，区别对待;

        // copy over the Jacobian (convert row-major -> column-major)
        if ( diffState )
        {
            _jacobianOplusXi = dError_dCamera;
            _jacobianOplusXj = dError_dPoint;
        }
        else
        {
            assert( 0 && "Error while differentiating" );
            _jacobianOplusXi.setZero();
            _jacobianOplusXj.setZero();
        }
    }
};



#endif // _VO_G2O_H_
