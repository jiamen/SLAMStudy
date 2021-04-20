
cmake_minimum_required( VERSION 2.8 )
project( pose_graph )


set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
# set(CMAKE_CXX_STANDARD 11)

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# Eigen
include_directories( "/usr/include/eigen3" )

# Sophus
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

# g2o
find_package( G2O REQUIRED )
include_directories( ${G2O_INCLUDE_DIRS} )
SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse )

find_package( Cholmod REQUIRED )
# include_directories( ${CHOLMOD_INCLUDE_DIRS} )    这里DIRS多了个S，所以程序是错的
include_directories( ${CHOLMOD_INCLUDE_DIR} )

# gtsam
find_package( GTSAMCMakeTools )
find_package( GTSAM REQUIRED )
include_directories( ${GTSAM_INCLUDE_DIR} )


# Ceres
find_package( Ceres REQUIRED )
include_directories( ${CERES_INCLUDE_DIR} )

add_executable( pose_graph_g2o_SE3 pose_graph_g2o_SE3.cpp )
target_link_libraries( pose_graph_g2o_SE3
        # g2o_core g2o_stuff g2o_types_slam3d
        ${G2O_LIBS}
        ${CHOLMOD_LIBRARIES}
        )


add_executable( pose_graph_gstam pose_graph_gtsam.cpp )
target_link_libraries( pose_graph_gstam
        ${CHOLMOD_LIBRARIES} gstam
)
