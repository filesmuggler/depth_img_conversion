#!/usr/bin/env python3
__author__ = 'Krzysztof Stezala <krzysztof.stezala at put.poznan.pl>'
__version__ = '0.1'
__license__ = 'MIT'

import sys
import rospy
import numpy as np
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
import os
import os.path
import yaml

class IpImg2Ros:
    """
    Publishes Depth image topic from series of PNG images
    """
    def __init__(self):
        rospy.loginfo("IpImg2Ros -> is RUN")
        self.depth = np.zeros([480,640])
        self.cv_bridge = CvBridge()
        self.rate = rospy.Rate(6)
        self.depth_publisher = rospy.Publisher("/ip_img_to_ros",Image, queue_size=10)
        self.cam_info_publisher = rospy.Publisher("/ip_img_to_ros_cam_info",CameraInfo,queue_size=10)
        self.imgs = []
        self.timestamps = []
        self.read_path_img, self.read_path_info = self.make_path_to_read()


    def pub_img(self):
        
        frames_file = open(self.read_path_info+"frames.txt", 'r')

        with open(self.read_path_info+"info.yaml",'r') as file_yaml:
            cam_info_config = yaml.safe_load(file_yaml)

        caminfo = CameraInfo()
        caminfo.header.seq = cam_info_config["header"]['seq']
        caminfo.header.stamp.secs = cam_info_config["header"]['stamp']['secs']
        caminfo.header.stamp.nsecs = cam_info_config["header"]['stamp']['nsecs']
        caminfo.header.frame_id = cam_info_config["header"]['frame_id']
        caminfo.height = cam_info_config["height"]
        caminfo.width = cam_info_config["width"]
        caminfo.distortion_model = cam_info_config["distortion_model"]
        caminfo.D = cam_info_config["D"]
        caminfo.K = cam_info_config["K"]
        caminfo.R = cam_info_config["R"]
        caminfo.P = cam_info_config["P"]
        caminfo.binning_x = cam_info_config["binning_x"]
        caminfo.binning_y = cam_info_config["binning_y"]
        caminfo.roi.x_offset = cam_info_config["roi"]['x_offset']
        caminfo.roi.y_offset = cam_info_config["roi"]['y_offset']
        caminfo.roi.height = cam_info_config["roi"]['height']
        caminfo.roi.width = cam_info_config["roi"]['width']
        caminfo.roi.do_rectify = cam_info_config["roi"]['do_rectify']

        
        


        filenames = frames_file.readlines()
        for f in filenames:
            ### IMAGE
            ## strip() removes newline character
            img_gray = cv2.imread(os.path.join(self.read_path_img,f.strip()),cv2.IMREAD_UNCHANGED )
            self.imgs.append(img_gray)

            ### TIMESTAMP
            time_s_ns = f.split(".")[0]
            t = rospy.Time()
            t.secs = int(time_s_ns.split("_")[0])
            t.nsecs = int(time_s_ns.split("_")[1])
            self.timestamps.append(t)

        counter = 0
        while not rospy.is_shutdown():
            
            msg = self.cv_bridge.cv2_to_imgmsg(self.imgs[counter], encoding="16UC1")
            msg.header.stamp = self.timestamps[counter]
            caminfo.header.seq = msg.header.seq
            msg.header.frame_id = caminfo.header.frame_id
            caminfo.header.stamp = self.timestamps[counter]
            
            #print(caminfo)

            self.depth_publisher.publish(msg)
            self.cam_info_publisher.publish(caminfo)
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


def main(args):
    rospy.init_node('ip_rosbag_2_img',anonymous=True)
    ip_i2r = IpImg2Ros()
    try:
        ip_i2r.pub_img()
    except KeyboardInterrupt:
        print("Shutting down ROS IpImg2Ros")

if __name__ == "__main__":
    main(sys.argv)