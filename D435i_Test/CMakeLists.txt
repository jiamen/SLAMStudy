
cmake_minimum_required( VERSION 2.8 )

project(d435i_test)

set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -O3")


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)


find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${realsense2_INCLUDE_DIRS})

add_executable(alldata GetAlldata.cpp)

target_link_libraries(alldata ${OpenCV_LIBS} ${realsense2_LIBRARY})

