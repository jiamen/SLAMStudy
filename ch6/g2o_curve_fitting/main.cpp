//
// Created by zlc on 2021/4/5.
//
#include <iostream>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
// 求解问题1：三个优化算法，使用哪种方法来定义定义非线性优化的下降策略
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
// 求解问题2：使用哪类线性求解器
#include <g2o/solvers/dense/linear_solver_dense.h>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <chrono>


using namespace std;


// 曲线模型的顶点，模板参数：优化变量维度和数据类型
class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 重置，估计系数（待优化的系数）重写虚函数
    // override保留字表示当前函数重写了基类的虚函数
    virtual void setToOriginImpl()  // 重置
    {
        _estimate << 0, 0, 0;       // 设置估计的系数
    }

    // 更新估计系数（待优化的系数）重写虚函数
    virtual void oplusImpl( const double* update )  // 更新
    {
        _estimate += Eigen::Vector3d(update);
    }

    // 存盘和读盘：留空
    virtual bool read( istream& in )  {  }
    virtual bool write( ostream& out ) const {  }
};


// 误差模型(边) 模板参数：观测值维度，类型，连接顶点类型
class CurveFittingEdge: public g2o::BaseUnaryEdge<1, double, CurveFittingVertex>
{
public:
    double _x;      // x值，y值为_measurement

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CurveFittingEdge( double x ) : BaseUnaryEdge(), _x(x) { }

    // 计算曲线模型误差
    void computeError()
    {
        const CurveFittingVertex* v = static_cast<const CurveFittingVertex*> (_vertices[0]);
        const Eigen::Vector3d abc = v->estimate();      // abc待优化的系数
        // 误差函数 =y-exp(ax2+bx+c)  这里面的abc都是待优化的系数
        _error(0, 0) = _measurement - std::exp( abc(0,0)*_x*_x + abc(1,0)*_x + abc(2,0) );
    }

    // 存盘和读盘：留空
    virtual bool read( istream& in )   { }
    virtual bool write( ostream& out ) const {  }
};


int main(int argc, char* *argv)
{
    double a=1.0, b=2.0, c=1.0;     // 真实参数值
    int N = 100;                    // 数据点
    double w_sigma = 1.0;           // 噪声Sigma值
    cv::RNG rng;                    // OpenCV随机数产生器
    double abc[3] = {0,0,0};        // abc参数的估计值


    vector<double> x_data, y_data;  // 数据

    cout << "generating data: " << endl;
    for ( int i=0; i<N; i ++ )
    {
        double x = i/100.0;
        x_data.push_back( x );
        y_data.push_back( exp(a*x*x + b*x +c) + rng.gaussian(w_sigma));
        cout << x_data[i] << " " << y_data[i] << endl;
    }

    // 第0步：构建图优化，先设定g2o块的大小
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<3,1> > Block;      // 每个误差项优化变量维度为3，误差值维度为1
    // 第1步：创建一个线性求解器LinearSolver
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();    // 线性方程求解器
    // 第2步：创建BlockSolver。并用上面定义的线性求解器初始化
    Block* solver_ptr = new Block( linearSolver );                      // 矩阵块求解器
    // 注意看g2O的结构图， BlockSolver由①SparseBlockMatrix<T>(Hessian矩阵) 和 ②LinearSolver（H△x=-b，线性方程求解器）
    // 所以Block的定义本身包含Hessian矩阵的块结构定义，同时使用线性方程求解器linearSolver进行初始化，由此 两方面内容 都进行定义说明。


    // 第3步：创建总求解器solver。并从从GN，LM，DogLeg中选一个作为梯度下降方法，再用上述BlockSolver初始化
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( solver_ptr );
    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr );
    // g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg( solver_ptr );

    // 第4步：创建终极大Boss  稀疏优化器（SparseOptimizer）
    g2o::SparseOptimizer optimizer;           // 图模型
    optimizer.setAlgorithm( solver );         // 设置求解器
    optimizer.setVerbose( true );    // 打开调试输出

    // 第5步：定义图的顶点和边，并往图中添加顶点
    CurveFittingVertex* v = new CurveFittingVertex();           // 设置顶点
    v->setEstimate( Eigen::Vector3d(0,0,0) );      // 设置估计值，这里相当于对待优化量abc没有初始估计值
    v->setId(0);             // 编号
    optimizer.addVertex( v );    // 往图中添加顶点

    for ( int i=0; i<N; i ++ )   // 往图中增加边
    {
        CurveFittingEdge* edge = new CurveFittingEdge( x_data[i] );
        edge->setId(i);          // 设置边的编号
        edge->setVertex(0, v);                // 设置连接的顶点，注意这里的0是顶点的序号
        edge->setMeasurement( y_data[i] );       // 观测数值
        edge->setInformation( Eigen::Matrix<double,1,1>::Identity() * 1/(w_sigma*w_sigma) );    // 信息矩阵：协方差矩阵之逆  1/(w_sigma * w_sigma)协方差矩阵之逆
        optimizer.addEdge( edge );               // 加入边
    }

    // 第6步：设置优化参数，开始执行优化
    cout << "start optimization" << endl;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(100);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used  = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
    cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<endl;

    // 输出优化值
    Eigen::Vector3d abc_estimate = v->estimate();
    cout << "estimated model: " << abc_estimate.transpose() << endl;

    return 0;
}
