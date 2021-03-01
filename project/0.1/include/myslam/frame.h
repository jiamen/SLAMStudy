//
// Created by zlc on 2021/2/28.
//

#ifndef _MYSLAM_FRAME_H_
#define _MYSLAM_FRAME_H_

#include "myslam/common_include.h"
#include "myslam/camera.h"

namespace myslam
{

// forward declare
class MapPoint;

class Frame
{
public:     // data members
    typedef std::shared_ptr<Frame>  Ptr;
    unsigned long                   id_;                // id of this frame                 ID
    double                          time_stamp_;        // when it is recorded           时间戳
    SE3                             T_c_w_;              // transform from world to camera  位姿
    Camera::Ptr                     camera_;            // PinHole RGBD Camera model   针孔模型
    Mat                             color_, depth_;     // color and depth image           图像

public:
    Frame();
    Frame( long id, double time_stamp=0, SE3 T_c_w=SE3(), Camera::Ptr= nullptr, Mat color=Mat(), Mat depth=Mat() );

    ~Frame();

    // factory function 创建帧
    static Frame::Ptr createFrame();

    // find the depth in depth map 寻找给定点对应的深度
    double findDepth( const cv::KeyPoint& kp );

    // Get Camera Center 获取相机光心
    Vector3d getCameraCenter() const;

    // check if a point is in this frame 判断某个点是否在视野内
    bool isInFrame( const Vector3d& pt_world );
};

}


#endif // _MYSLAM_FRAME_H_
