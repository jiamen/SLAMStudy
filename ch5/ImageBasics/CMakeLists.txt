
cmake_minimum_required( VERSION 2.8 )

project( imageBasics )

set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# 寻找OpenCV库
find_package( OpenCV 3 REQUIRED )

MESSAGE(STATUS "OpenCV_DIR = ${OpenCV_DIR}")
MESSAGE(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
MESSAGE(STATUS "OpenCV_LIBS = ${OpenCV_LIBS}")

# 添加头文件
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( imageBasics imageBasics.cpp )
target_link_libraries( imageBasics ${OpenCV_LIBS} )

add_executable( imageDistorted imageDistorted.cpp )
target_link_libraries( imageDistorted ${OpenCV_LIBS} )


