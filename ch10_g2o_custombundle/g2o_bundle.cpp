//
// Created by zlc on 2021/5/11.
//

#include <Eigen/StdVector>
#include <Eigen/Core>

#include <iostream>
#include <stdint.h>

#include <unordered_set>
#include <memory>
#include <vector>
#include <stdlib.h>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/robust_kernel_impl.h"
// 非线性优化方法
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_dogleg.h"

#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/dense/linear_solver_dense.h"


// 解析出参数
#include "common/BundleParams.h"
#include "common/BALProblem.h"      // g2o问题定义
#include "g2o_bal_class.h"

using namespace Eigen;
using namespace std;


typedef Eigen::Map<Eigen::VectorXd> VectorRef;
typedef Eigen::Map<const Eigen::VectorXd> ConstVectorRef;
typedef g2o::BlockSolver<g2o::BlockSolverTraits<9,3>> BalBlockSolver;

// set up the vertexes and edges for the bundle adjustment.
void BuildProblem(const BALProblem* bal_problem, g2o::SparseOptimizer* optimizer, const BundleParams& params)
{
    const int num_points  = bal_problem->num_points();
    const int num_cameras = bal_problem->num_cameras();     // 16
    const int camera_block_size = bal_problem->camera_block_size();
    const int point_block_size  = bal_problem->point_block_size();

    // Set camera vertex with initial value in the dataset   设置相机位姿顶点
    const double* raw_cameras = bal_problem->cameras();     // 获得 相机位姿 各自参数 的首地址
    for (int i=0; i<num_cameras; i ++)
    {
        ConstVectorRef temVecCamera(raw_cameras + camera_block_size*i, camera_block_size);
        VertexCameraBAL* pCamera = new VertexCameraBAL();   // 创建准备存放的相机优化顶点

        pCamera->setEstimate(temVecCamera);             // initial value for the camera i ...
        pCamera->setId(i);                                  // set id for each camera vertex

        // remember to add vertex into optimizer ...
        optimizer->addVertex(pCamera);
    }

    // Set point vertex with initial value in the dataset.  设置路标点 顶点
    const double* raw_points = bal_problem->points();       // 获得 特征点 各自参数 的首地址
    // const int point_block_size = bal_problem->point_block_size();
    for (int j=0; j<num_points; j ++)
    {
        ConstVectorRef temVecPoint(raw_points+point_block_size*j, point_block_size);
        VertexPointBAL* pPoint = new VertexPointBAL();      // 创建准备存放的路标优化顶点
        pPoint->setEstimate(temVecPoint);                   // initial value for the point i ...
        pPoint->setId(j + num_cameras);                 // each vertex should have an unique id, no matter it is a camera vertex, or a point vertex

        pPoint->setMarginalized(true);          // 必须全部进行Schur消元，原因见课本《14讲》P264

        // remember to add vertex into optimizer ..
        optimizer->addVertex(pPoint);
    }

    // Set edges for graph ...
    const int num_observations = bal_problem->num_observations();
    const double* observations = bal_problem->observations();       // pointer for the first observation ...

    for (int i=0; i<num_observations; i ++)
    {
        EdgeObservationBAL* bal_edge = new EdgeObservationBAL();

        const int camera_id = bal_problem->camera_index()[i];               // get id for the camera
        const int point_id  = bal_problem->point_index()[i] + num_cameras;  // get id for the point

        // 是否使用鲁棒核函数
        if (params.robustify)
        {
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            rk->setDelta(1.0);
            bal_edge->setRobustKernel(rk);
        }

        // set the vertex by the ids for an edge observation
        bal_edge->setVertex(0, dynamic_cast<VertexCameraBAL*>(optimizer->vertex(camera_id)));
        bal_edge->setVertex(1, dynamic_cast<VertexPointBAL*>(optimizer->vertex(point_id)));
        bal_edge->setInformation(Eigen::Matrix2d::Identity());
        bal_edge->setMeasurement(Eigen::Vector2d(observations[2*i+0], observations[2*i+1]));

        optimizer->addEdge(bal_edge);
    }
}


// 从给定的优化器参数中设置好求解器
void SetSolverOptionsFromFlags(BALProblem* bal_problem, const BundleParams& params, g2o::SparseOptimizer* optimizer)
{
    BalBlockSolver* solver_ptr;

    // ① 设置求解方法
    g2o::LinearSolver<BalBlockSolver::PoseMatrixType>* linearSolver = 0;
    // 稠密舒尔补
    cout << params.linear_solver << endl;
    if (params.linear_solver == "dense_schur")
    {
        linearSolver = new g2o::LinearSolverDense<BalBlockSolver::PoseMatrixType>();            // 默认是稠密舒尔补
    }
    else if (params.linear_solver == "sparse_schur")
    {
        linearSolver = new g2o::LinearSolverCholmod<BalBlockSolver::PoseMatrixType>();
        // AMD ordering , only needed for sparse cholesky solver
        dynamic_cast< g2o::LinearSolverCholmod<BalBlockSolver::PoseMatrixType>* >(linearSolver)->setBlockOrdering(true);
    }

    solver_ptr = new BalBlockSolver(linearSolver);
    // SetLinearSolver(solver_ptr, params);

    // ② 选择非线性优化方式，方式不同会得到不同的迭代公式  H△x = g
    // SetMinimizerOptions(solver_ptr, params, optimizer);
    g2o::OptimizationAlgorithmWithHessian* solver;
    if (params.trust_region_strategy == "levenberg_marquardt")
    {
        solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    }
    else if (params.trust_region_strategy == "dogleg")
    {
        solver = new g2o::OptimizationAlgorithmDogleg(solver_ptr);
    }
    else
    {
        std::cout << "Please check your trust_region_strategy parameter again.." << std::endl;
        exit(EXIT_FAILURE);
    }

    optimizer->setAlgorithm(solver);
}


// 将优化后的点云进行存储
void WriteToBALProblem(BALProblem* bal_problem, g2o::SparseOptimizer* optimizer)
{
    const int num_points  = bal_problem->num_points();      // 路标点 数量
    const int num_cameras = bal_problem->num_cameras();     // 相机   数量
    const int camera_block_size = bal_problem->camera_block_size();     // 相机位姿块大小
    const int point_block_size  = bal_problem->point_block_size();      // 路标点大小

    double* raw_cameras = bal_problem->mutable_cameras();
    for (int i=0; i<num_cameras; ++ i)
    {
        VertexCameraBAL* pCamera = dynamic_cast<VertexCameraBAL*> (optimizer->vertex(i));
        Eigen::VectorXd NewCameraVec = pCamera->estimate();
        memcpy(raw_cameras + i*camera_block_size, NewCameraVec.data(), sizeof(double)*camera_block_size);
    }

    // parameters中  点的位置
    double* raw_points = bal_problem->mutable_points();
    for (int j=0; j<num_points; j ++)
    {
        VertexPointBAL* pPoint = dynamic_cast<VertexPointBAL*>( optimizer->vertex(j + num_cameras) );
        Eigen::Vector3d NewPointVec = pPoint->estimate();
        memcpy(raw_points + j * point_block_size, NewPointVec.data(), sizeof(double) * point_block_size);
    }

}


// BA问题求解总流程
void SolveProblem(const char* filename, const BundleParams& params)
{
    // 第一步：定义BA问题
    BALProblem bal_problem(filename);

    // show some information here ...
    std::cout << "bal problem file loaded ... " << std::endl;
    std::cout << "bal problem have " << bal_problem.num_cameras() << " cameras and "
              << bal_problem.num_points() << " points." << std::endl;
    std::cout << "Forming " << bal_problem.num_observations() << " observations." << std::endl;     // 观测点

    // store the initial 3D cloud points and camera pose .  存储初始3D点云和相机位姿
    if (!params.initial_ply.empty())
    {
        bal_problem.WriteToPLYFile(params.initial_ply);
    }

    std::cout << "beginning problem ..." << std::endl;

    // add some noise for the initial value
    srand(params.random_seed);                          // 随机数
    bal_problem.Normalize();                            // 对数据进行归一化
    bal_problem.Perturb(params.rotation_sigma, params.translation_sigma, params.point_sigma);       //给数据加上噪声（相机旋转、相机平移、路标点）

    std::cout << "Normalization complete ..." << std::endl;

    // 第二步：从给定的优化器参数中设置好求解器
    g2o::SparseOptimizer optimizer;
    SetSolverOptionsFromFlags(&bal_problem, params, &optimizer);
    // 第三步：构建BA优化问题，设置点和边
    BuildProblem(&bal_problem, &optimizer, params);

    std::cout << "begin optimization ..." << std::endl;

    // 第四步：执行BA求解   perform the optimization
    optimizer.initializeOptimization();
    optimizer.setVerbose(true);                 // 打开输出调试
    optimizer.optimize(params.num_iterations);          // 设置优化次数,执行优化

    std::cout << "optimization complete..." << std::endl;
    WriteToBALProblem(&bal_problem, &optimizer);

    // write the result into a .plu file.   将优化后的点云存储下来，与之前未优化的点云进行对比
    if (!params.final_ply.empty())
    {
        bal_problem.WriteToPLYFile(params.final_ply);
    }
}


int main(int argc, char* *argv)
{
    // set the parameters here.
    BundleParams params(argc, argv);

    if ( params.input.empty() )
    {
        std::cout << "Usage: bundle_adjuster -input <path for dataset>";
        return 1;
    }

    SolveProblem(params.input.c_str(), params);

    return 0;
}



