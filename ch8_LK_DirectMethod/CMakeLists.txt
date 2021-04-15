
cmake_minimum_required( VERSION 2.8 )
project( ch8 )

set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# 添加cmake模块路径
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package( OpenCV )
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package( G2O )
include_directories( ${G2O_INCLUDE_DIRS} )

include_directories( "/usr/include/eigen3" )

MESSAGE(STATUS "OpenCV_DIR = ${OpenCV_DIR}")
MESSAGE(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
MESSAGE(STATUS "OpenCV_LIBS = ${OpenCV_LIBS}")


add_executable( useLK useLK.cpp )
target_link_libraries( useLK ${OpenCV_LIBS} )


# set( G2O_LIBS
#     g2o_core g2o_types_sba g2o_solver_csparse g2o_stuff g2o_csparse_extension
# )
SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d
        g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse )


add_executable( direct_sparse direct_sparse_semidense.cpp)
target_link_libraries( direct_sparse ${OpenCV_LIBS} ${G2O_LIBS} )



