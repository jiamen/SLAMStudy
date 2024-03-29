//
// Created by zlc on 2021/2/26.
//

#include "DBoW3/DBoW3.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <iostream>
#include <vector>
#include <string>

using namespace cv;
using namespace std;


int main(int argc, char* *argv)
{
    // read the images and database.
    cout << "reading database" << endl;
    DBoW3::Vocabulary vocab("./vocabulary.yml.gz");
    // DBoW3::Vocabulary vocab( "./vocabulary_larger.yml.gz" );

    if( vocab.empty() )
    {
        cerr << "Vocabulary does not exist." << endl;
        return 1;
    }
    cout << vocab << endl;       // Vocabulary: k = 10, L = 5, Weighting = tf-idf, Scoring = L1-norm, Number of words = 4970

    /* 这里的图像是待查询图像 */
    cout << "reading images ..." << endl;
    vector<Mat> images;
    for ( int i=0; i<10; i ++ )
    {
        string path = "../data/" + to_string(i+1) + ".png";
        images.push_back( imread(path) );
    }

    // NOTE: in this case we are comparing images with a vocabulary generated by themselves, this may leed to overfitting.
    // detect ORB features
    cout << "detecting ORB features ... " << endl;
    Ptr<Feature2D> detector = ORB::create();
    vector<Mat> descriptors;
    for ( Mat& image:images )       // 注意这里的图像是所有的待查询图像
    {
        vector<KeyPoint> keypoints;
        Mat descriptor;
        detector->detectAndCompute( image, Mat(), keypoints, descriptor );
        descriptors.push_back(descriptor);
    }


    // We can compare the images directly or we can compare one image to a database
    // images : 直接对比待查询图像
    cout << "comparing images with images: " << endl;
    for ( int i=0; i<images.size(); i ++ )
    {
        DBoW3::BowVector v1;
        vocab.transform( descriptors[i], v1 );      // 将由待查询图像得到的描述子进行转换，转换为词袋向量
        for( int j=i; j<images.size(); j ++ )
        {
            DBoW3::BowVector v2;
            vocab.transform( descriptors[j], v2 );
            double score = vocab.score(v1, v2);
            cout << "image " << i+1 << " vs image " << j+1 << " : " << score << endl;
        }
        cout << endl;
    }


    // or with database
    cout << "comparing images with database: " << endl;
    DBoW3::Database db( vocab, false, 0 );      // 把所有的单词放入数据库db中, 但是还没有建立查询向量
    cout << "before database info: " << db << endl;
    // before database info: Database: Entries = 0, Using direct index = no. Vocabulary: k = 10, L = 5,
    // Weighting = tf-idf, Scoring = L1-norm, Number of words = 4970
    for( int i=0; i<descriptors.size(); i ++ )
    {
        db.add(descriptors[i]);     // 将每张图片的所有特征子添加到数据库中，此处应该含着将图片描述子descriptors转换为词袋向量word的过程，根据特征子生成查询词袋bag-of-words
    }
    cout << "after database info: " << db << endl;
    // database info: Database: Entries = 10, Using direct index = no. Vocabulary: k = 10, L = 5,
    // Weighting = tf-idf, Scoring = L1-norm, Number of words = 4970

    for( int i=0; i<descriptors.size(); i ++ )
    {
        DBoW3::QueryResults ret;
        db.query( descriptors[i], ret, 4 );     // max result = 4  给定特征值，返回相似度值最大的前4个结果
        cout << "searching for image " << i << " returns " << ret << endl << endl;
    }
    cout << "done." << endl;
    cout << "descriptors.size()： " << descriptors.size() << endl;       // 10

    return 0;
}
