#!/usr/bin/env python3

import rospkg
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest, SetBool, SetBoolResponse
import rospy
import os
import cv2
import numpy as np
import warnings
warnings.filterwarnings("ignore")

class Collect():
    def __init__(self):
        self.bridge = CvBridge()

        # Mssage filter
        self.color_right = message_filters.Subscriber('/camera_mid/color/image_raw', Image)
        self.depth_right = message_filters.Subscriber('/camera_mid/aligned_depth_to_color/image_raw', Image)

        ts = message_filters.TimeSynchronizer([self.color_right, self.depth_right], 10)
        ts.registerCallback(self.callback_msgs)
        self.num = 0
        print("start")

    def callback_msgs(self, colorR, depthR):

        self.color_right = colorR
        self.depth_right = depthR
        cv_image = self.bridge.imgmsg_to_cv2(self.color_right, "bgr8")
        cv_depth = self.bridge.imgmsg_to_cv2(self.depth_right, "16UC1")

        cv2.imwrite('/home/arg-vx300s/Pick-and-Place-with-RL/d435/rgb/'+str(self.num)+'.png', cv_image)
        cv2.imwrite('/home/arg-vx300s/Pick-and-Place-with-RL/d435/depth/'+str(self.num)+'.png', cv_depth)
        print('/home/arg-vx300s/Pick-and-Place-with-RL/d435/rgb/'+str(self.num)+'.png')
        
        self.num += 1

if __name__ == '__main__':
    rospy.init_node("collect", anonymous=False)
    collect = Collect()
    rospy.spin()