//
// Created by zlc on 2021/2/28.
//

#ifndef _MYSLAM_CONFIG_H_
#define _MYSLAM_CONFIG_H_


#include "common_include.h"

namespace myslam
{

class Config
{
private:
    static std::shared_ptr<Config> config_;
    cv::FileStorage file_;

    Config() {  }   // private constructor makes a singleton

public:
    ~Config();      // 析构函数close the file when deconstructing

    // set a new config file
    static void setParameterFile( const std::string& filename );

    // access the parameter values
    template< typename T >
    static T get( const std::string& key )
    {
        return T( Config::config_->file_[key] );
    }

};

}

#endif // _MYSLAM_CONFIG_H_
