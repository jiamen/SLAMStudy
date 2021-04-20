//
// Created by zlc on 2021/3/27.
//

// #include <librealsense2/rs.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

// include OpenCV/header file
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;


#define width 640
#define height 480
#define fps 30



//获取深度像素对应长度单位（米）的换算比例
float get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}


int main(int argc, char* *argv)
{
    // judge whether devices is exist or not
    rs2::context ctx;

    auto list = ctx.query_devices();        // Get a snapshot of currently connected devices
    try
    {
        if( 0 == list.size() )
            throw std::runtime_error("No device detected. is it plugged in?");

    }
    // error
    catch(const rs2::error& e)
    {
        std::cerr << "RealSense error calling" << e.get_failed_function() << "(" << e.get_failed_args() << "):\n"
                  << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;     // what():  out of range value for argument "index"
    }
    rs2::device dev = list.front();             // 找到插入的设备D435i


    rs2::frameset frames;     // D435i一帧所有数据
    // 创建一个通信管道 //https://baike.so.com/doc/1559953-1649001.html pipeline的解释
    rs2::pipeline pipe;
    // 创建一个以非默认配置的配置项用来配置管道
    rs2::config cfg;

    // 向配置项添加所需的流
    cfg.enable_stream(RS2_STREAM_COLOR,width,height,RS2_FORMAT_BGR8,fps);
    cfg.enable_stream(RS2_STREAM_DEPTH,width,height,RS2_FORMAT_Z16,fps);
    cfg.enable_stream(RS2_STREAM_INFRARED,1,width,height,RS2_FORMAT_Y8, fps);
    cfg.enable_stream(RS2_STREAM_INFRARED,2,width,height,RS2_FORMAT_Y8, fps);
    cfg.enable_stream(RS2_STREAM_ACCEL,RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO,RS2_FORMAT_MOTION_XYZ32F);


    // get depth scale  获取深度像素对应长度单位（米）的换算比例
    float depth_scale = get_depth_scale(dev);
    cout <<  " depth_scale: " << depth_scale << endl;       //  depth_scale: 0.001

    // start stream
    pipe.start(cfg);    // 指示管道使用所请求的配置启动流


    while(1)
    {
        frames = pipe.wait_for_frames();        // 等待

        // Align to depth
        rs2::align align_to_depth(RS2_STREAM_DEPTH);
        frames = align_to_depth.process(frames);

        // Get imu data
        if( rs2::motion_frame accel_frame=frames.first_or_default(RS2_STREAM_ACCEL) )
        {
            rs2_vector accel_sample = accel_frame.get_motion_data();
            std::cout << "Accel: " << accel_sample.x << ", " << accel_sample.y << ", " << accel_sample.z << std::endl;
        }
        if( rs2::motion_frame gyro_frame=frames.first_or_default(RS2_STREAM_GYRO) )
        {
            rs2_vector gyro_sample = gyro_frame.get_motion_data();
            std::cout << "Gyro: " << gyro_sample.x << ", " << gyro_sample.y << ", " << gyro_sample.z << std::endl;
        }
        // Accel: -0.12231, -9.91462, 0.812859
        // Gyro: -0.0034311, 4.02672e-06, 1.23899e-06

        // Get each frame
        rs2::frame color_frame = frames.get_color_frame();
        rs2::frame depth_frame = frames.get_depth_frame();
        rs2::video_frame ir_frame_left  = frames.get_infrared_frame(1);
        rs2::video_frame ir_frame_right = frames.get_infrared_frame(2);


        // Creating OpenCV Matrix from a color image
        Mat color(Size(width, height), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
        Mat pic_left(Size(width, height), CV_8UC1, (void*)ir_frame_left.get_data());
        Mat pic_right(Size(width, height), CV_8UC1, (void*)ir_frame_right.get_data());
        Mat pic_depth(Size(width, height), CV_16U, (void*)depth_frame.get_data(), Mat::AUTO_STEP);


        // Display in a GUI
        namedWindow("Display Image", WINDOW_AUTOSIZE );
        imshow("Display Image", color);
        waitKey(1);
        imshow("Display depth", pic_depth*15);
        waitKey(1);
        imshow("Display pic_left", pic_left);
        waitKey(1);
        imshow("Display pic_right",pic_right);
        waitKey(1);
    }


    return 0;
}


