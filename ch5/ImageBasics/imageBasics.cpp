//
// Created by zlc on 2021/5/6.
// Description： 图像基本操作：图像加载，显示，像素访问，拷贝，修改

#include <iostream>
#include <chrono>

using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char* *argv)
{
    // 读取argv[1]指定的图像
    cv::Mat image;
    image = cv::imread( "../ubuntu.png" );      // cv::imread函数读取指定路径下的图像
    // 判断图像是否正确读取
    if (nullptr == image.data)
    {
        cerr << "文件" << "ubuntu.png" << "不存在" << endl;
        return 0;
    }

    // 文件顺序读取，首先输出一些基本信息
    cout << "图像宽为" << image.cols << ", 高为" << image.rows << ", 通道数为" <<image.channels() << endl;
    cv::imshow("image", image);     // 用cv::imshow显示图像
    cv::waitKey(0);                         // 程序暂停，等待按键输入

    // 判断image的类型
    if (image.type() != CV_8UC1 && image.type() != CV_8UC3)
    {
        // 图像类型不符合要求
        cout << "请输入一张彩色图或灰度图." << endl;
        return 0;
    }

    // 遍历图像，请注意以下遍历方式亦可使用于随机像素访问
    // 使用std::chrono 来给算法计时
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    for (size_t y=0; y<image.rows; y ++)
    {
        // 用cv::Mat::ptr获得图像的行指针
        unsigned char* row_ptr = image.ptr<unsigned char> (y);          // row_ptr是第y行的头指针
        for (size_t x=0; x<image.cols; x ++)
        {
            // 访问位于x，y处的像素
            unsigned char* data_ptr = &row_ptr[x*image.channels()];     // data_ptr指向待访问的像素数据
            for (size_t c=0; c!=image.channels(); c ++)
            {
                unsigned char data = data_ptr[c];                       // data为I(x,y)第c个通道的值
            }
        }
    }
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2 - t1 );
    cout << "遍历图像用时：" << time_used.count() << "秒." << endl;


    // 关于cv::Mat的拷贝
    // 直接赋值并不会拷贝数据
    cv::Mat image_another = image;
    // 修改 image_another 会导致 image 发生变化
    image_another (cv::Rect(0,0,100,100)).setTo(0);     // 将左上角100*100的块置零

    cv::imshow("image", image);
    cv::waitKey(0);

    // 使用clone函数来拷贝数据
    cv::Mat image_clone = image.clone();
    image_clone (cv::Rect(0,0,100,100)).setTo(255);    // 将左上角100*100的块置零,//255代表RGB255 截取的区域是白色，左上角显示白色
    cv::imshow("image", image);
    cv::imshow("image_clone", image_clone);
    cv::waitKey(0);

    // 对于图像还有很多基本的操作，如剪切，旋转，缩放等

    cv::destroyAllWindows();

    return 0;
}


