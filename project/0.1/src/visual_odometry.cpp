//
// Created by zlc on 2021/2/28.
//

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include <boost/timer.hpp>


#include "../include/myslam/config.h"
#include "../include/myslam/visual_odometry.h"


namespace myslam
{

// 默认构造函数，提供默认值、读取配置参数
VisualOdometry::VisualOdometry() :
    state_(INITIALIZING), ref_(nullptr), curr_(nullptr), map_(new Map), num_lost_(0), num_inliers_(0)
{
    num_of_features_ = Config::get<int>("number_of_features");
    scale_factor_       = Config::get<double> ( "scale_factor" );
    level_pyramid_      = Config::get<int> ( "level_pyramid" );
    match_ratio_        = Config::get<float> ( "match_ratio" );
    max_num_lost_       = Config::get<float> ( "max_num_lost" );
    min_inliers_        = Config::get<int> ( "min_inliers" );
    key_frame_min_rot_  = Config::get<double> ( "keyframe_rotation" );
    key_frame_min_trans_= Config::get<double> ( "keyframe_translation" );
    // map_point_erase_ratio_ = Config::get<double> ( "map_point_erase_ratio" );

    orb_ = cv::ORB::create( num_of_features_, scale_factor_, level_pyramid_ );  // 两个参数用于创建orb对象

}

VisualOdometry::~VisualOdometry()
{

}


bool VisualOdometry::addFrame(Frame::Ptr frame)
{
    switch ( state_ )
    {
        case INITIALIZING:
        {
            state_ = OK;
            curr_ = ref_ = frame;
            map_->insertKeyFrame( frame );
            // extract features from first frame
            extractKeyPoints();
            computeDescriptors();
            // compute the 3d position of features in ref frame
            setRef3DPoints();
            break;
        }

        case OK:
        {
            curr_ = frame;
            extractKeyPoints();         // 提取orb特征点
            computeDescriptors();       // 计算特征点描述子
            featureMatching();          // orb特征匹配
            poseEstimationPnP();        // 相机运动估计
            if( checkEstimatedPose() == true )      // 这里要检查位姿,通过点数和
            {
                // T_c_w_ = T_c_r * T_r_w;       T_r_w = ref_->T_c_w_
                curr_->T_c_w_ = T_c_r_estimated_ * ref_->T_c_w_;
                ref_ = curr_;
                setRef3DPoints();
                num_lost_ = 0;      // 丢失跟踪帧数清0
                if( checkKeyFrame() == true )
                {
                    addKeyFrame();
                }
            }
            else    // bad estimation due to various resons
            {
                num_lost_ ++;
                if( num_lost_ > max_num_lost_ )
                {
                    state_ = LOST;
                }
                return false;
            }
            break;
        }

        case LOST:
        {
            cout << "vo has lost." << endl;
            break;
        }
    }

    return true;
}



// 提取orb特征点
void VisualOdometry::extractKeyPoints()
{
    boost::timer timer;
    orb_->detect( curr_->color_, keypoints_curr_);      // 保存当前帧中的关键点
    cout << "extract keypoints cost time: " << timer.elapsed() << endl;
}

// 计算orb特征描述子
void VisualOdometry::computeDescriptors()
{
    boost::timer timer;
    orb_->compute( curr_->color_, keypoints_curr_, descriptors_curr_ );
    cout << "descriptor computation cost time: " << timer.elapsed() << endl;
}

// orb特征匹配
void VisualOdometry::featureMatching()
{
    boost::timer timer;
    vector<cv::DMatch> matches;
    // 从参考帧的对应的地图点中参与匹配的点：必须在当前帧的视野内
    /*Mat desp_map;
    vector<MapPoint::Ptr> candidate;
    for( auto& allpoints:map_->map_points_ )
    {
        MapPoint::Ptr& p = allpoints.second;
        // 判断点p是否在当前帧图像范围内
        if( curr_->isInFrame(p->pos_) )
        {
            // add to candidate
            p->visible_times_
        }
    }*/

    cv::BFMatcher matcher( cv::NORM_HAMMING );
    matcher.match( descriptors_ref_, descriptors_curr_, matches );

    // select the best matches
    float min_dis = std::min_element(
            matches.begin(), matches.end(),
            []( const cv::DMatch& m1, const cv::DMatch& m2 )
            {
                return m1.distance < m2.distance;
            }
    )->distance;

    feature_matches_.clear();
    for( cv::DMatch& m : matches )
    {
        if( m.distance <  max<float>(min_dis*match_ratio_, 30.0) )
        {
            feature_matches_.push_back(m);
        }
    }
    cout << "good matches: " << feature_matches_.size() << endl;
}


// 设置参考的3D点
void VisualOdometry::setRef3DPoints()
{
    // select the features with depth measurements
    pts_3d_ref_.clear();
    descriptors_ref_ = Mat();

    for( size_t i=0; i<keypoints_curr_.size(); i ++ )
    {
        double d = ref_->findDepth(keypoints_curr_[i]); // 找到关键点对应的深度
        if( d > 0 )
        {
            Vector3d p_cam = ref_->camera_->pixel2camera(
                Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y), d
            );

            pts_3d_ref_.push_back( cv::Point3f( p_cam(0,0), p_cam(1,0), p_cam(2,0) ) );
            descriptors_ref_.push_back(descriptors_curr_.row(i) );
        }
    }
}


void VisualOdometry::poseEstimationPnP()
{
    // construct the 3d 2d observations
    vector<cv::Point3f> pts3d;
    vector<cv::Point2f> pts2d;

    for( cv::DMatch m : feature_matches_ )
    {
        pts3d.push_back( pts_3d_ref_[m.queryIdx] );
        pts2d.push_back( keypoints_curr_[m.trainIdx].pt );
    }

    // 构建相机内参矩阵
    Mat K = ( cv::Mat_<double>(3, 3) <<
            ref_->camera_->fx_, 0, ref_->camera_->cx_,
            0, ref_->camera_->fy_, ref_->camera_->cy_,
            0, 0, 1
    );

    // 两帧之间的旋转向量和平移向量由PnP得到
    Mat rvec, tvec, inliers;
    cv::solvePnPRansac( pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers );

    num_inliers_ = inliers.rows;
    cout << "pnp inliers: " << num_inliers_ << endl;
    T_c_r_estimated_ = SE3(
        SO3(rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0)),
        Vector3d( tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0) )
    );
}



// 对估计的位姿进行检查：
// 1:内点数量, 2:移动不能过大
bool VisualOdometry::checkEstimatedPose()
{
    // check if the estimared pose is good
    if( num_inliers_ < min_inliers_ )
    {
        cout << "reject because inlier is too small: " << num_inliers_ << endl;
        return false;
    }

    // if the motion is too large, it is probably wrong
    // 课本P216页， Tcw = Tcr*Trw    Tcw*Trw^(-1) = Tcr    Trc=Tcr^(-1)=Trw*Tcw^(-1)
    // Trw= T_c_w_  Tcw^(-1)= T_c_w_estimated_.inverse()
    // SE3 T_r_c = ref_->T_c_w_ * T_c_w_estimated_.inverse();  // 当前帧和参考帧之间的变化
    Sophus::Vector6d d = T_c_r_estimated_.log();
    if ( d.norm() > 5.0 )
    {
        cout << "reject because motion is too large: " << d.norm() << endl;
        return false;
    }

    return true;
}


// 检查是否可以成为关键帧
bool VisualOdometry::checkKeyFrame()
{
    Sophus::Vector6d d = T_c_r_estimated_.log();
    Vector3d trans = d.head<3>();
    Vector3d rot = d.tail<3>();
    if( rot.norm() > key_frame_min_rot_ || trans.norm() > key_frame_min_trans_ )
    {
        return true;
    }
    return false;
}

void VisualOdometry::addKeyFrame()
{
    cout << "adding a key-frame" << endl;
    map_->insertKeyFrame( curr_ );
}

}


