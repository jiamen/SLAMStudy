//
// Created by zlc on 2021/2/28.
//

#ifndef _MYSLAM_VISUAL_ODOMETRY_H_
#define _MYSLAM_VISUAL_ODOMETRY_H_

#include "myslam/common_include.h"
#include "myslam/map.h"

#include <opencv2/features2d/features2d.hpp>


/**
 * 只关心两个帧之间的运动估计,并且不优化特征点的位置。然而把估得的位姿“串”起来,也能得到一条运动轨迹。
 * 这种方式可以看成两两帧间的(Pair-wise),无结构(Structureless)的 VO。
 */

namespace myslam
{

class VisualOdometry
{
public:
    // 定义指向自身的智能指针，在后面传递参数是使用VisualOdometry::Ptr即可
    typedef shared_ptr<VisualOdometry> Ptr;
    // 定义枚举来表征VO状态，分别为：初始化，正常，丢失
    enum VOState
    {
        INITIALIZING=-1,
        OK=0,
        LOST
    };

    // 这里为两两帧VO所用到的参考帧和当前帧。还有VO状态和整个地图。
    VOState  state_;        // current VO status
    Map::Ptr map_;          // map with all frames and map points.
    Frame::Ptr ref_;        // reference key-frame
    Frame::Ptr curr_;       // current frame


    // 这里是两帧匹配需要的：keypoints, descriptors, matches,
    // 相比0.2去掉了关于参考帧的东西, 3D点，描述子等
    cv::Ptr<cv::ORB> orb_;                      // orb detector and computer
    vector<cv::Point3f>  pts_3d_ref_;           // 3d points in reference frame
    vector<cv::KeyPoint> keypoints_curr_;       // 当前帧中的关键点
    Mat                  descriptors_ref_;      // 参考帧中的关键描述子
    Mat                  descriptors_curr_;     // descriptor in current frame
    vector<cv::DMatch>   feature_matches_;


    // 在匹配器中，所需要的匹配变成了地图点和帧中的关键点
    // cv::FlannBasedMatcher matcher_flann_;       // flann matcher
    // vector<MapPoint::Ptr>   match_3dpts_;       // matched 3d points
    // vector<int>        match_2dkp_index_;       // matched 2d pixels(index of kp_curr)

    // 这里为匹配结果T，还有表征结果好坏的内点数和丢失数
    SE3 T_c_r_estimated_;       // the estimated pose of current frame
    int num_inliers_;           // number of inlier features in icp
    int num_lost_;              // number of lost times

    // parameters
    int num_of_features_;       // number of features 特征数量
    double scale_factor_;       // scale in image pyramid
    int   level_pyramid_;       // number of pyramid levels
    float   match_ratio_;       // ratio for selecting good matches    好匹配比率
    int    max_num_lost_;       // max number of continuous lost times.   最大允许的连续丢失时间
    int     min_inliers_;       // minimum inliers      最小内点数

    // 用于判定是否为关键帧, 说白了就是大于一定幅度的旋转和平移，大于这个幅度就归为关键帧
    double key_frame_min_rot_;          // minimal rotation of two key-frames
    double key_frame_min_trans_;        // minimal translation of two key-frames
    // double map_point_erase_ratio_;   // remove map point ratio


public:
    VisualOdometry();
    ~VisualOdometry();

    // △△△ 这个函数为核心处理函数，将帧添加进来，然后处理 △△△
    bool  addFrame(Frame::Ptr frame);       // add a new frame


protected:
    // inner operation  一些内部处理函数, 这块主要是特征匹配的
    void extractKeyPoints();
    void computeDescriptors();
    void featureMatching();
    void poseEstimationPnP();
    void setRef3DPoints();

    // 增加的优化地图的函数，这个函数可能实现的就是整个后端地图的优化
    void optimizeMap();

    // 这里是关键帧的一些功能函数
    // 增加地图点函数
    void addKeyFrame();         // void Map::insertKeyFrame(Frame::Ptr frame)
    void addMapPoints();        // void Map::insertMapPoint(MapPoint::Ptr map_point)
    bool checkEstimatedPose();
    bool checkKeyFrame();

    // 增加的取得视角函数
    double getViewAngle( Frame::Ptr frame, MapPoint::Ptr point );
};

}


#endif // _MYSLAM_VISUAL_ODOMETRY_H_
