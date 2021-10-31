#!/usr/bin/env python3

__author__ = 'Krzysztof Stezala <krzysztof.stezala at put.poznan.pl>'
__version__ = '0.1'
__license__ = 'MIT'

import sys
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
import os
import os.path

class IpRosbag2Img:
    """
    Saves Depth image topic into series of PNG images
    """
    def __init__(self):
        rospy.loginfo("IpRosbag2Img -> is RUN")
        self.save_path_img, self.save_path_info = self.make_path_to_save()
        self.depth = np.zeros([480,640])
        self.cam_info = CameraInfo()
        self.cv_bridge = CvBridge()
        self.depth_subscriber = rospy.Subscriber("/depth_camera_front/depth/image_rect_raw",Image, self.callback_depth, queue_size=1)
        self.info_subscriber = rospy.Subscriber("/depth_camera_front/depth/camera_info",CameraInfo, self.callback_info, queue_size=1)
        
        ## clears file
        open(self.save_path_info+"frames.txt", 'w').close()
        
    def make_path_to_save(self):
        bag_path = rospy.get_param("~path_to_save")
        parts = bag_path.split(".")
        new_path_img = parts[0] + "/images/"
        new_path_info = parts[0] + "/info/"
        if os.path.exists(new_path_img):
            pass
        else:
            os.makedirs(new_path_img)
        if os.path.exists(new_path_info):
            pass
        else:
            os.makedirs(new_path_info)

        return new_path_img, new_path_info

    def callback_depth(self,data):
        # filter image elementwise numpy
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(data, "16UC1")
            self.depth = cv_image
            self.depth_timestamp = data.header.stamp

            filename = str(self.depth_timestamp.secs) + "_" + str(self.depth_timestamp.nsecs) + ".png"
            frames_file = open(self.save_path_info+"frames.txt","a")
            frames_file.write(filename+"\n")
            frames_file.close()

            filename_path = self.save_path_img + str(self.depth_timestamp.secs) + "_" + str(self.depth_timestamp.nsecs) + ".png"

            cv2.imwrite(filename_path,self.depth.astype(np.uint16))
        except CvBridgeError as e:
            print("cv bridge: ",e)

    def callback_info(self,data):
        """
        Saves camera info once
        """
        self.cam_info = data
        file = open(self.save_path_info+"info.yaml","w")
        file.write(str(self.cam_info))
        file.close()

        self.info_subscriber.unregister()

def main(args):
    rospy.init_node('ip_rosbag_2_img',anonymous=True)
    ip_r2i = IpRosbag2Img()
    print(rospy.get_param("~path_to_save"))
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS IpRos2Img")

if __name__ == "__main__":
    main(sys.argv)