//
// Created by zlc on 2021/2/28.
//

#include "../include/myslam/frame.h"

namespace myslam
{

Frame::Frame()
: id_(-1), time_stamp_(-1), camera_(nullptr)
{

}

Frame::Frame( long id, double time_stamp, SE3 T_c_w, Camera::Ptr camera, Mat color, Mat depth )
: id_(id), time_stamp_(time_stamp), T_c_w_(T_c_w), camera_(camera), color_(color), depth_(depth)
{

}

Frame::~Frame()
{

}


// 工厂模式创建帧，每一帧都有自己的编号
Frame::Ptr Frame::createFrame()
{
    static long factory_id = 0;
    return Frame::Ptr( new Frame(factory_id ++) );
}


// 给定坐标在深度图中找到对应的深度
double Frame::findDepth( const cv::KeyPoint& kp )
{
    int x = cvRound(kp.pt.x);
    int y = cvRound(kp.pt.y);
    ushort d = depth_.ptr<ushort>(y)[x];            // depth 是 Mat类型
    if( 0 != d )
    {
        return double(d)/camera_->depth_scale_;
    }
    else
    {
        // check the nearby points
        int dx[4] = {-1, 0, 1, 0};
        int dy[4] = {0, -1, 0, 1};          // dx，dy上下连起来看，正好是左上右下四个周围点
        for( int i=0; i<4; i ++ )
        {
            d = depth_.ptr<ushort>(y+dy[i])[x+dx[i]];
            if( 0 != d )    // 若周围点深度不为0，则直接返回周围点深度
            {
                return double(d) / camera_->depth_scale_;
            }
        }
    }

    return -1.0;
}


Vector3d Frame::getCameraCenter() const
{
    // 这里一定要想明白相机的光心在相机坐标系下就是（0，0，0），
    // 所以求变换矩阵的逆之后，直接求对应的平移矩即R^(-1)*(-t)
    return T_c_w_.inverse().translation();
}


// 判断点是不是在当前帧中，主要有两点判断：
// 1.判断深度信息
// 2.判断像素是否超过该照片的范围。
bool Frame::isInFrame(const Vector3d &pt_world)
{
    Vector3d p_cam = camera_->world2camera( pt_world, T_c_w_ );
    // 这步是取得Z值，Z为相机和物体实际距离，小于0直接return false;
    if( p_cam(2, 0) < 0 )
        return false;


    Vector2d pixel = camera_->world2pixel( pt_world, T_c_w_ );
    // xy值都大于0并且小于color图的行列
    // color的信息，就是一张照片的像素点，
    // pixel是一个二维向量而已，它的值其实是该点的坐标值，
    // 所以说将世界坐标系中的点转换到//照片上来说，
    // 要注意该像素点的坐标是否在照片上，是否超过了该照片的范围，
    // 即color.cols和color.rows.}
    return pixel(0,0) > 0 && pixel(1,0) > 0
            && pixel(0,0) < color_.cols
            && pixel(1,0) < color_.rows;
}

}

