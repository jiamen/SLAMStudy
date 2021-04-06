# -*- coding: utf-8 -*-
# 获取摄像头的内参是为了将像素坐标转化成实际坐标
#

import pyrealsense2 as rs

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

profile = pipeline.start(config)        # 使用默认的推荐配置启动流式传输
frames = pipeline.wait_for_frames()
depth = frames.get_depth_frame()
color = frames.get_color_frame()

# 1、获取深度图相关
depth_profile = depth.get_profile()
print (depth_profile)                    # <pyrealsense2.video_stream_profile: Depth(0) 640x480 @ 30fps Z16>
print (type(depth_profile))              # <class 'pyrealsense2.pyrealsense2.stream_profile'>

print (depth_profile.fps())              # 30
print (depth_profile.stream_index())     # 0
print (depth_profile.fps())              # 30
print (depth_profile.stream_name())      # Depth
print (depth_profile.stream_type())      # stream.depth
print (depth_profile.unique_id)
# <bound method stream_profile.unique_id of <pyrealsense2.video_stream_profile: Depth(0) 640x480 @ 30fps Z16>>


print ('\n')


# 2、获取彩色图相关
color_profile = color.get_profile()
print (color_profile)                   # <pyrealsense2.video_stream_profile: Color(0) 640x480 @ 30fps BGR8>
print (type(color_profile))             # <class 'pyrealsense2.pyrealsense2.stream_profile'>
print (color_profile.fps())             # 30
print (depth_profile.stream_index())    # 0


# 3、获取视频流参数
cvs_profile = rs.video_stream_profile(color_profile)
dvs_profile = rs.video_stream_profile(depth_profile)

# 最重要的内参矩阵K获取方法
color_intrinsic = cvs_profile.get_intrinsics()
print (color_intrinsic)
# [ 640x480  p[316.419 245.025]  f[603.525 604.052]  Inverse Brown Conrady [0 0 0 0 0] ]

depth_intrinsic = dvs_profile.get_intrinsics()
print (depth_intrinsic)
# [ 640x480  p[325.793 238.33]  f[381.801 381.801]  Brown Conrady [0 0 0 0 0] ]


extrinsic = depth_profile.get_extrinsics_to(color_profile)
print (extrinsic)
# rotation: [0.999931, 0.0117314, 2.3913e-05, -0.0117314, 0.999931, 0.00036152, -1.96702e-05, -0.000361776, 1]
# translation: [0.0148373, 0.000557109, 0.000161708]

