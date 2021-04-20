//
// Created by zlc on 2021/3/1.
//

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
// 本模块为跨平台的gui/IO组件，支持平台包括windows,linux,mac,IOS,android，
// 可支持图像/视频/摄像头的读取显示以及转码。
#include <opencv2/calib3d/calib3d.hpp>


using namespace std;
using namespace cv;

/* 本程序演示如何使用2D-2D的特征匹配估计相机运动 */


void find_feature_matches ( const Mat& img_1, const Mat& img_2,
                            std::vector<KeyPoint>& keypoints_1,
                            std::vector<KeyPoint>& keypoints_2,
                            std::vector< DMatch >& matches );

// 使用对极约束计算变换关系R，t
void pose_estimation_2d2d(
    std::vector<KeyPoint> keypoints_1,
    std::vector<KeyPoint> keypoints_2,
    std::vector< DMatch > matches,
    Mat& R, Mat& t);


// 像素坐标转相机归一化坐标
Point2d pixel2cam( const Point2d& p, const Mat& K );

int main( int argc, char* *argv )
{
    if( 3 != argc )
    {
        cout << "usage: pose_estimation_2d2d img1 img2" << endl;
        return 1;
    }

    // 读取图像
    Mat img_1 = imread( argv[1], CV_LOAD_IMAGE_COLOR );
    Mat img_2 = imread( argv[2], CV_LOAD_IMAGE_COLOR );

    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches ( img_1, img_2, keypoints_1, keypoints_2, matches );
    cout << "一共找到了 " << matches.size() << " 组匹配点." << endl;


    // 估计两张图像间运动
    Mat R, t;
    pose_estimation_2d2d( keypoints_1, keypoints_2, matches, R, t );



    // 验证E=t^R*scale  P65页反对称矩阵变换
    Mat t_x = ( Mat_<double>(3, 3) <<
                0,                          -t.at<double>(2,0),  t.at<double>(1,0),
                t.at<double>(2,0),   0,                         -t.at<double>(0,0),
                -t.at<double>(1,0),  t.at<double>(0,0),   0);
    cout << "t^R = " << endl << t_x*R << endl;

    // 验证対极约束
    Mat K = ( Mat_<double>(3,3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
    int i = 0;
    for( DMatch m : matches )
    {
        Point2d pt1 = pixel2cam( keypoints_1[ m.queryIdx ].pt, K );      // 像素坐标转归一化坐标
        Mat y1 = ( Mat_<double> (3,1) << pt1.x, pt1.y, 1 );   // 得到特征点的归一化坐标
        Point2d pt2 = pixel2cam( keypoints_2[ m.trainIdx ].pt, K );
        Mat y2 = ( Mat_<double> (3,1) << pt2.x, pt2.y, 1 );
        Mat d = y2.t() * t_x * R * y1;                  // 《SLAM》 P143页 7.9式（3）
        cout << "i: " << ++ i << " epipolar constraint = " << d << endl;      // 输出対极约束
    }

    return 0;
}

// 找到两张图像中匹配的特征点
void find_feature_matches ( const Mat& img_1, const Mat& img_2,
                            std::vector<KeyPoint>& keypoints_1,
                            std::vector<KeyPoint>& keypoints_2,
                            std::vector< DMatch >& matches )
{
    //-- 1、初始化
    Mat descriptors_1, descriptors_2;
    // used in OpenCV3
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    // use this if you are in OpenCV2
    // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );
    //-- 2、检测 Oriented FAST 角点位置
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    //-- 3、根据角点位置计算 BRIEF 描述子
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    //-- 4、对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> match;
    //BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_1, descriptors_2, match );

    //-- 5、匹配点对筛选
    double min_dist=10000, max_dist=0;

    // 找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = match[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( match[i].distance <= max ( 2*min_dist, 30.0 ) )
        {
            matches.push_back ( match[i] );
        }
    }
}

// 像素坐标系到相机坐标系
Point2d pixel2cam( const Point2d& p, const Mat& K )
{
    return Point2d
           (
           ( p.x - K.at<double>(0,2) ) / K.at<double>(0,0),
           ( p.y - K.at<double>(1,2) ) / K.at<double>(1,1)
           );
    // 《SLAM14讲》 P86 (X/Y) = (u-cx) / fx
}


// 使用对极约束计算变换关系R，t
void pose_estimation_2d2d( std::vector<KeyPoint> keypoints_1,
                           std::vector<KeyPoint> keypoints_2,
                           std::vector< DMatch > matches,
                           Mat& R, Mat& t )
{
    // 相机内参， TUM Freiburg2
    Mat K = ( Mat_<double> (3,3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );

    // 把匹配点转换为vector<Point2f>的形式
    vector<Point2f> points1;
    vector<Point2f> points2;

    for ( int i=0; i<(int)matches.size(); i ++ )    // 81组匹配点
    {
        points1.push_back( keypoints_1[matches[i].queryIdx].pt );
        points2.push_back( keypoints_2[matches[i].trainIdx].pt );
    }
    // queryIdx : 查询点的索引（当前要寻找匹配结果的点在它所在图片上的索引）.
    // trainIdx : 被查询到的点的索引（存储库中的点的在存储库上的索引）
    // imgIdx : 有争议(常为0)

    // 计算基础矩阵：基础矩阵是根据 像素坐标系 下的像素直接计算
    Mat fundamental_matrix;
    fundamental_matrix = findFundamentalMat( points1, points2, CV_FM_8POINT );
    cout << "fundamental_matrix is " << endl << fundamental_matrix << endl;


    // 计算本质矩阵 ：根据 相机坐标系 下的 坐标 计算，需要像素坐标转换
    Point2d principal_point(325.1, 249.7);        // 相机光心， TUM dataset标定值，内参矩阵K中的cx和cy
    double focal_length = 521;                          // 相机焦距， TUM dataset标定值
    Mat essential_matrix;
    essential_matrix = findEssentialMat( points1, points2, focal_length, principal_point );
    cout << "essential_matrix is " << endl << essential_matrix << endl;


    // 计算单应矩阵：单应矩阵描述处于共同平面上的一些点在两张图像之间的变换关系，作为基础矩阵的补充
    Mat homography_matrix;
    homography_matrix = findHomography( points1, points2, RANSAC, 3 );
    cout << "homography_matrix is " << endl << homography_matrix << endl;


    // 从本质矩阵中恢复旋转和平移信息
    recoverPose( essential_matrix, points1, points2, R, t, focal_length, principal_point );
    cout << "R is " << endl << R << endl;
    cout << "t is " << endl << t << endl;
}

