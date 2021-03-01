//
// Created by zlc on 2021/2/28.
//

#ifndef _MYSLAM_COMMON_INCLUDE_H_
#define _MYSLAM_COMMON_INCLUDE_H_


// 定义常用的头文件, 防止其他文件有长的头文件
// for Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
using Eigen::Vector2d;
using Eigen::Vector3d;


// for Sophus
#include <sophus/se3.h>
#include <sophus/so3.h>
using Sophus::SE3;
using Sophus::SO3;


// for cv
#include <opencv2/core/core.hpp>
using cv::Mat;


// std
#include <vector>
#include <list>
#include <memory>
#include <string>
#include <iostream>
#include <set>
#include <unordered_map>
#include <map>

using namespace std;

#endif // _MYSLAM_COMMON_INCLUDE_H_
