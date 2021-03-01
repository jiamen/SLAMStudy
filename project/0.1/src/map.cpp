//
// Created by zlc on 2021/2/28.
//

#include "../include/myslam/map.h"


namespace myslam
{

void Map::insertKeyFrame(Frame::Ptr frame)
{
    cout << "Key frame size = " << keyframes_.size() << endl;

    // 现有的关键帧中没有当前帧，则在keyframes中插入
    if( keyframes_.find(frame->id_) == keyframes_.end() )
    {
        keyframes_.insert( make_pair(frame->id_, frame) );
        // make_pair把帧号和帧绑定在一起
    }
    else
    {
        keyframes_[frame->id_] = frame;     // 找到了就更新关键帧
    }
}

void Map::insertMapPoint(MapPoint::Ptr map_point)
{
    if( map_points_.find(map_point->id_) == map_points_.end() )
    {
        map_points_.insert( make_pair(map_point->id_, map_point) );
    }
    else
    {
        map_points_[map_point->id_] = map_point;    // 找到了就更新关键点
    }
}


}

