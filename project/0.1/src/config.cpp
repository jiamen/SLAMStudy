//
// Created by zlc on 2021/2/28.
//

#include "../include/myslam/config.h"

namespace myslam
{

// 文件读取，使用OpenCVtigong的FileStorage类。它可以读取一个YAML文件，且可以访问任意一个字段。
void Config::setParameterFile(const std::string &filename)
{
    if( nullptr == config_ )
    {
        config_ = std::shared_ptr<Config>( new Config );
    }

    config_->file_ = cv::FileStorage( filename.c_str(), cv::FileStorage::READ );

    // 参数文件打不开则报错
    if( false == config_->file_.isOpened() )
    {
        std::cerr << "parameter file " << filename << "does not exist." << std::endl;
        config_->file_.release();

        return;
    }
}

// 析构函数把打开的文件释放
Config::~Config()
{
    if( file_.isOpened() )
        file_.release();
}


// 实际构造的对象是Config的智能指针
// 用智能指针的原因是可以自动析构，省得我们再调用一个其他的函数来做析构
std::shared_ptr<Config> Config::config_ = nullptr;

}

