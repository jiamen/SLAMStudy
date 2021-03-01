//
// Created by zlc on 2021/2/28.
//

#ifndef _MYSLAM_MAPPOINT_H_
#define _MYSLAM_MAPPOINT_H_

#include "common_include.h"

namespace myslam
{

class Frame;
class MapPoint
{
public:
    typedef shared_ptr<MapPoint> Ptr;
    unsigned long                id_;               // ID    特征点ID
    Vector3d                     pos_;              // Position in world  真实世界中的位置
    Vector3d                     norm_;             // Normal of viewing direction                           观察方向法线
    Mat                          descriptor_;       // Descriptor for matching       当前帧提取到的特征点与地图中的路标点匹配
    int                          observed_times_;   // being observed by feature matching algo.  被特征匹配算法观察到的时间
    int                          correct_time_;     // being an inliner in pose estimation       位姿估计时的时间

    MapPoint();
    MapPoint( long id, Vector3d position, Vector3d norm );

    // factory function
    static MapPoint::Ptr createMapPoint();
};

}

#endif // _MYSLAM_MAPPOINT_H_
