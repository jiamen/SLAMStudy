//
// Created by zlc on 2021/5/9.
//

#ifndef G2O_CUSTOMBUNDLE_PROJECTION_H
#define G2O_CUSTOMBUNDLE_PROJECTION_H

#include "tools/rotation.h"

// camera : 9 dimes array with
// [0-2]： angle-axis rotation
// [3-5]： translation
// [6-8]： camera parameter， [6] focal length, [7-8] second and forth order radial distortion.   k1,k2
// point : 3D location.
// predictions : 2D predictions with center of the image plane.

/*
 * 本文件实现了带有畸变的相机投影过程，其中使用了相机参数camera，是一个9维的列表，将point点云转化为重投影预测值predictions
 * （是以图像中心作为坐标原点，并非以左上角，倒像取负，并非用的正面归一化平面坐标）。
 * */

template <typename T>
inline bool CamProjectionWithDistortion(const T* camera, const T* point, T* predictions)
{
    // Rodrigues's formula
    T p[3];

    // point 通过 camera相机位姿转换到 相机坐标系下，得到点 p
    AngleAxisRotatePoint(camera, point, p);

    // camera[3,4,5] are the translation    旋转后 + 平移，得到相机坐标系下的值
    p[0] += camera[3];
    p[1] += camera[4];
    p[2] += camera[5];

    // Compute the center for distortion    得到相机坐标系下的归一化坐标系
    T xp = -p[0] / p[2];
    T yp = -p[1] / p[2];

    // Apply second and fourth order radial distortion
    const T& l1 = camera[7];
    const T& l2 = camera[8];

    T r2 = xp*xp + yp*yp;
    T distortion = T(1.0) + l1 * r2 + l2 * r2*r2;       // 畸变校正系数，P88页， 式5.11

    // 这个模型直接以图像中心作为坐标原点，并非以左上角为左边原点
    const T& focal = camera[6];
    predictions[0] = focal * distortion * xp;
    predictions[1] = focal * distortion * yp;

    return true;
}


#endif //G2O_CUSTOMBUNDLE_PROJECTION_H
