//
// Created by zlc on 2021/5/9.
//

/*
 * 这整个类的功能就是对原始的txt数据进行分割存储，然后提供对txt数据的读取写入和生成PLY文件功能。
   存储部分：相机维度，路标点维度，相机个数，路标点个数，观测值个数，待优化参数个数（相机维度相机个数+点维度点个数）mutable存储优化后数据（可变）
 * */


#ifndef _G2O_CUSTOMBUNDLE_BALPROBLEM_H_
#define _G2O_CUSTOMBUNDLE_BALPROBLEM_H_

#include <stdio.h>
#include <string>
#include <iostream>


class BALProblem
{
public:
    explicit BALProblem(const std::string& filename, bool use_quaternions=false);

    ~BALProblem()
    {
        delete[] point_index_;
        delete[] camera_index_;
        delete[] observations_;
        delete[] parameters_;
    }

    void WriteToFile(const std::string& filename) const;
    void WriteToPLYFile(const std::string& filename) const;

    void Normalize();

    void Perturb(const double rotation_sigma,
                 const double translation_sigma,
                 const double point_sigma);

    // 根据旋转是否使用四元数判断，相机参数块维度是10还是9
    int camera_block_size ()        const { return use_quaternions_? 10 : 9; }
    int point_block_size()          const { return 3;                        }

    int num_cameras()               const { return num_cameras_;             }
    int num_points()                const { return num_points_;              }
    int num_observations()          const { return num_observations_;        }
    int num_parameters()            const { return num_parameters_;          }

    const int* point_index()        const { return point_index_;             }
    const int* camera_index()       const { return camera_index_;            }
    const double* observations()    const { return observations_;            }
    const double* parameters()      const { return parameters_;              }

    // 返回数据中相机位姿数据列的开头位置        获得相机和点各自参数的首地址
    const double* cameras()         const { return parameters_;              }
    const double* points()          const { return parameters_ + camera_block_size() * num_cameras_; }

    // mutable 可变的
    double* mutable_cameras()             { return parameters_;              }
    double* mutable_points()              { return parameters_ + camera_block_size() * num_cameras_ ; }

    // 返回第i个观测中的相机数据
    double* mutable_camera_for_observation(int i)
    {
        return mutable_cameras() + camera_index_[i] * camera_block_size();
    }

    // 返回第i个观测中的路标数据
    double* mutable_point_for_observation(int i)
    {
        return mutable_cameras() + point_index_[i] * point_block_size();
    }

    const double* camera_for_observation(int i) const
    {
        return cameras() + camera_index_[i] * camera_block_size();
    }

    const double* point_for_observation(int i) const
    {
        return points() + point_index_[i] * point_block_size();
    }


private:
    // 相机转换到轴角以及中心点
    void CameraToAngelAxisAndCenter(const double* camera,
                                    double* angle_axis,
                                    double* center) const;

    // 轴角和中心点转换到相机
    void AngleAxisAndCenterToCamera(const double* angle_axis,
                                    const double* center,
                                    double* camera) const;

    int num_cameras_;               // 相机位姿个数
    int num_points_;                // 特征点（路标点）数量
    int num_observations_;           // 观测点数量
    int num_parameters_;            // 参数数量
    bool use_quaternions_;

    int* point_index_;              // 所有特征点序列
    int* camera_index_;             // 所有相机位姿序列
    double* observations_;          // 相机得到的观测点，观测的索引数组
    double* parameters_;            // 参数
};



#endif // _G2O_CUSTOMBUNDLE_BALPROBLEM_H_
