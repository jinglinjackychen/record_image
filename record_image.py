#!/usr/bin/env python3

import numpy as np
import cv2
import rospy
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Empty, EmptyResponse

cnt = 0
image_rgb = None
image_depth = None
info_rgb = None
lock = False
br = CvBridge()

def callback(rgb_image, rgb_info, depth_image):
    global image_rgb, image_depth, info_rgb

    if not lock:
        image_rgb = br.imgmsg_to_cv2(rgb_image, "bgr8")
        # image_rgb = cv2.cvtColor(image_rgb, cv2.COLOR_BGR2RGB)
        image_depth = br.imgmsg_to_cv2(depth_image, "16UC1")
        info_rgb = rgb_info
		
def save_imgs_cb(req):
    global cnt
    lock = True
    cv2.imwrite("d435/color_{:04d}.png".format(cnt), image_rgb)
	# print image_depth.dtype
    cv2.imwrite("d435/depth_{:04d}.png".format(cnt), image_depth)
    # np.save("d435/depth_{:04d}.npy".format(cnt), image_depth)
    fs = open("d435/rgb_intrinsic_file.txt", "w")
    info_str = "rgb_intrinsic: [" + str(info_rgb.K[0]) + ", " + str(info_rgb.K[4]) + ", " + str(info_rgb.K[2]) + ", " + str(info_rgb.K[5]) + "]\n"
    fs.write(info_str)
    fs.close()
	
    cnt += 1
    lock = False
    print('Images saved')
    return EmptyResponse()

rgb_image_sub     = message_filters.Subscriber("camera_mid/color/image_raw", Image)
rgb_info_sub      = message_filters.Subscriber("camera_mid/color/camera_info", CameraInfo)
depth_image_sub   = message_filters.Subscriber("camera_mid/aligned_depth_to_color/image_raw", Image)

rospy.init_node("save_images_server")
ts = message_filters.ApproximateTimeSynchronizer([rgb_image_sub, rgb_info_sub, depth_image_sub], 10, 0.2)
ts.registerCallback(callback)

s = rospy.Service("save_image", Empty, save_imgs_cb)

while not rospy.is_shutdown():
    rospy.spin()