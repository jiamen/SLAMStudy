
cmake_minimum_required( VERSION 2.8 )
project( draw_trajectory )

set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# 添加Eigen库
include_directories("/usr/include/eigen3")

# 添加Sophus库
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

# 添加OpenCV依赖
find_package( OpenCV 3 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# 添加Pangolin依赖
find_package( Pangolin REQUIRED )
include_directories( ${Pangolin_INCLUDE_DIRS} )
MESSAGE(STATUS "Pangolin_DIR = ${Pangolin_DIR}")
MESSAGE(STATUS "Pangolin_INCLUDE_DIRS = ${Pangolin_INCLUDE_DIRS}")
MESSAGE(STATUS "Pangolin_LIBRARIES = ${Pangolin_LIBRARIES}")

# QUITE 参数指代，当查找出错时，屏蔽 message() 的输出。
# REQUIRED 参数指代，当找不到 package 时，终止过程。


add_executable( draw_trajectory draw_trajectory.cpp )
target_link_libraries( draw_trajectory ${OpenCV_LIBS} ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES})

