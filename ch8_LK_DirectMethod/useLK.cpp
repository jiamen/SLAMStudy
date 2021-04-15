//
// Created by zlc on 2021/4/11.
//

#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <chrono>

using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>


int main(int argc, char* *argv)
{
    if (argc != 2)
    {
        cout << "usage: useLK path_to_dataset" << endl;
        return 1;
    }
    string path_to_dataset = argv[1];
    string associate_file = path_to_dataset + "/associate.txt";

    ifstream fin( associate_file );
    if ( !fin )
    {
        cerr << "I can't find associate.txt!" << endl;
        return 1;
    }

    string rgb_file, depth_file, time_rgb, time_depth;
    list< cv::Point2f > keypoints;          // 因为要删除跟踪失败的点，使用list
    cv::Mat color, depth, last_color;

    for (int index=0; index<9; index ++)    // 这里默认数据集中就提供了前9张图
    {
        fin >> time_rgb >> rgb_file >> time_depth >> depth_file;
        color = cv::imread( path_to_dataset+"/"+rgb_file );
        depth = cv::imread( path_to_dataset+"/"+depth_file, -1 );

        if ( 0 == index )
        {
            // “只”对第一帧提取FAST特征点
            vector<cv::KeyPoint> kps;
            cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create();
            detector->detect(color, kps);
            for (auto kp:kps)
                keypoints.push_back(kp.pt);

            last_color = color;

            continue;
        }

        if ( color.data == nullptr || depth.data == nullptr )
            continue;

        // 对其他帧用LK跟踪特征点
        vector<cv::Point2f> next_keypoints;         // 即将计算得出的当前帧图像的特征点
        vector<cv::Point2f> prev_keypoints;         // 保存已有的上一帧的特征点
        for ( auto kp:keypoints )
            prev_keypoints.push_back(kp);

        vector<unsigned char> status;       // 标记跟踪失败与否的状态，跟踪失败后 点 对应的status=0
        vector<float> error;

        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

        cv::calcOpticalFlowPyrLK(last_color, color, prev_keypoints, next_keypoints, status, error);

        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
        cout << "LK Flow use time: " << time_used.count() << " seconds." << endl;

        // 在keypoints中把跟丢的点删掉,每次都从keypoint中删除，因为第一帧keypoints是最多的，后面随着跟丢逐渐减少
        int i = 0;
        for (auto iter=keypoints.begin(); iter!=keypoints.end(); i ++)      // 循环写的有点意思
        {
            if (0 == status[i])
            {
                iter = keypoints.erase(iter);   // 清除原keypoints点的空间，并返回该点指针位置,并且个数减1
                continue;
            }
            *iter = next_keypoints[i];          // 覆盖keypoints中的点，重新写入next_keypoints中跟踪成功的点
            iter ++;
        }


        cout << "tracked keypoints: " << keypoints.size() << endl;
        if (0 == keypoints.size())
        {
            cout << "all keypoints are lost." << endl;
            break;
        }

        // 画出keypoints
        cv::Mat img_show = color.clone();
        for ( auto kp:keypoints)
            cv::circle(img_show, kp, 10, cv::Scalar(0, 240, 0), 1);
        cv::imshow("corners", img_show);
        cv::waitKey(0);

        last_color = color;
        printf("iter = %d\n ", index);
    }

    return 0;
}



