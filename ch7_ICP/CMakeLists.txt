
cmake_minimum_required(VERSION 2.8)

project(vo_icp)


set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# 添加cmake模块以使用g2o
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )


# Found OpenCV: /usr/local (found suitable version "3.4.3", minimum required is "3.1")
find_package( OpenCV 3.1 REQUIRED )
find_package( G2O REQUIRED )
find_package( CSparse REQUIRED )


include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${CSPARSE_INCLUDE_DIR}
        "/usr/include/eigen3/"
)


# add_library(libHello src/Hello.cpp)

add_executable( pose_estimate_3d3d pose_estimate_3d3d.cpp )
target_link_libraries( pose_estimate_3d3d
        ${OpenCV_LIBS}
        g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
        ${CSPARSE_LIBRARY}
)

# target_link_libraries( sayhello libHello )








