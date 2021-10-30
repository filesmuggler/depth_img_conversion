#!/usr/bin/env python3
"""
Saves Depth image topic into series of PNG images
"""
__author__ = 'Krzysztof Stezala <krzysztof.stezala at put.poznan.pl>'
__version__ = '0.1'
__license__ = 'MIT'

import sys,time
import rospy
import roslib
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
import os
import os.path
from os import path
from PIL import Image as PImage

class IpImg2Ros:
    def __init__(self):
        rospy.loginfo("IpImg2Ros -> is RUN")
        self.depth = np.zeros([480,640])
        self.cv_bridge = CvBridge()
        self.rate = rospy.Rate(6)
        self.depth_publisher = rospy.Publisher("/depth_camera_front/depth/ip_img_to_ros",Image, queue_size=10)
        self.imgs = []
        self.timestamps = []
        self.read_path_img, self.read_path_info = self.make_path_to_read()


    def pub_img(self):
        frames_file = open(self.read_path_info+"frames.txt", 'r')
        filenames = frames_file.readlines()
        for f in filenames:
            img_gray = cv2.imread(os.path.join(self.read_path_img,f.strip()),cv2.IMREAD_UNCHANGED )

            self.imgs.append(img_gray)
            time_s_ns = f.split(".")[0]
            t = rospy.Time()
            t.secs = int(time_s_ns.split("_")[0])
            t.nsecs = int(time_s_ns.split("_")[1])
            self.timestamps.append(t)

        counter = 0
        while not rospy.is_shutdown():
            
            msg = self.cv_bridge.cv2_to_imgmsg(self.imgs[counter], encoding="16UC1")
            msg.header.stamp = self.timestamps[counter]
            
            self.depth_publisher.publish(msg)
            self.rate.sleep()
            counter = counter + 1
            if counter >= len(self.imgs):
                break
            


    def make_path_to_read(self):
        bag_path = rospy.get_param("~path_to_read")
        parts = bag_path.split(".")
        path_img = parts[0] + "/images/"
        path_info = parts[0] + "/info/"
        return path_img, path_info

    def callback_depth(self,data):
        # filter image elementwise numpy
        try:
            # cv_image = self.cv_bridge.imgmsg_to_cv2(data, "32FC1")
            # cv_image_array = nanp.array(cv_image, dtype = np.dtype('f4'))
            # #cv_image_array = cv_image_array * 255.0
            # # Normalize the depth image to fall between 0 (black) and 1 (white)            
            # cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
            # self.depth = cv_image_norm


            cv_image = self.cv_bridge.imgmsg_to_cv2(data, "16UC1")
            self.depth = cv_image
            self.depth_timestamp = data.header.stamp

            filename = self.save_path + str(self.depth_timestamp) + ".png"

            cv2.imwrite(filename,self.depth)
            # print(self.depth_timestamp)
            # cv2.namedWindow('image')
            # cv2.imshow("image", self.depth)
            # cv2.waitKey(3)
        except CvBridgeError as e:
            print("cv bridge: ",e)

def main(args):
    rospy.init_node('ip_rosbag_2_img',anonymous=True)
    ip_i2r = IpImg2Ros()
    try:
        ip_i2r.pub_img()
    except KeyboardInterrupt:
        print("Shutting down ROS IpImg2Ros")
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main(sys.argv)