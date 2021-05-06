
cmake_minimum_required( VERSION 2.8 )
project( disparity )

set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# 添加Eigen库
include_directories("/usr/include/eigen3")

# 添加OpenCV依赖
find_package( OpenCV 3 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# 添加Pangolin依赖
find_package( Pangolin )
include_directories( ${Pangolin_INCLUDE_DIRS} )

add_executable( disparity disparity.cpp )
target_link_libraries( disparity ${OpenCV_LIBS} ${Pangolin_LIBRARIES})
