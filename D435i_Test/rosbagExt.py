import roslib
import rosbag
import rospy
import cv2
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

rgb = '/home/zlc/Documents/rosImage/rgb/'  	  #rgb path
depth = '/home/zlc/Documents/rosImage/depth/'   #depth path
bridge = CvBridge()

file_handle1 = open('/home/zlc/Documents/rosImage/depth-stamp.txt', 'w')
file_handle2 = open('/home/zlc/Documents/rosImage/rgb-stamp.txt', 'w')

with rosbag.Bag('/home/zlc/Documents/room.bag', 'r') as bag:
    for topic, msg, t in bag.read_messages():
        if topic == "/device_0/sensor_0/Depth_0/image/data":  #depth topic
            cv_image = bridge.imgmsg_to_cv2(msg)
            timestr = "%.6f" % msg.header.stamp.to_sec()   #depth time stamp
            image_name = timestr + ".png"
            path = "depth/" + image_name
            file_handle1.write(timestr + " " + path + '\n')
            cv2.imwrite(depth + image_name, cv_image)
        if topic == "/device_0/sensor_1/Color_0/image/data":   #rgb topic
            cv_image = bridge.imgmsg_to_cv2(msg,"bgr8")
            timestr = "%.6f" % msg.header.stamp.to_sec()   #rgb time stamp
            image_name = timestr + ".jpg"
            path = "rgb/" + image_name
            file_handle2.write(timestr + " " + path + '\n')
            cv2.imwrite(rgb + image_name, cv_image)
file_handle1.close()
file_handle2.close()
