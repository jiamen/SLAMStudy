//
// Created by zlc on 2021/2/28.
//

#ifndef _MYSLAM_MAP_H_
#define _MYSLAM_MAP_H_

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/mappoint.h"

namespace myslam
{

class Map
{
public:
    typedef shared_ptr<Map> Ptr;
    unordered_map<unsigned long, MapPoint::Ptr> map_points_;    // all landmarks
    unordered_map<unsigned long, Frame::Ptr>     keyframes_;    // all key-frames

    Map()  {  }

    void insertKeyFrame( Frame::Ptr frame );            // 插入关键帧
    void insertMapPoint( MapPoint::Ptr map_point );     // 插入关键点
};

}

#endif // _MYSLAM_MAP_H_
