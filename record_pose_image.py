#!/usr/bin/env python3

import yaml
import time
import tf
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest, SetBool, SetBoolResponse
import rospy
import cv2
import numpy as np
import warnings
warnings.filterwarnings("ignore")
import tf.transformations as tr

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3

import numpy as np

def pose_to_pq(msg):
    """Convert a C{geometry_msgs/Pose} into position/quaternion np arrays

    @param msg: ROS message to be converted
    @return:
      - p: position as a np.array
      - q: quaternion as a numpy array (order = [x,y,z,w])
    """
    p = np.array([msg.position.x, msg.position.y, msg.position.z])
    q = np.array([msg.orientation.x, msg.orientation.y,
                  msg.orientation.z, msg.orientation.w])
    return p, q


def pose_stamped_to_pq(msg):
    """Convert a C{geometry_msgs/PoseStamped} into position/quaternion np arrays

    @param msg: ROS message to be converted
    @return:
      - p: position as a np.array
      - q: quaternion as a numpy array (order = [x,y,z,w])
    """
    return pose_to_pq(msg.pose)


def transform_to_pq(msg):
    """Convert a C{geometry_msgs/Transform} into position/quaternion np arrays

    @param msg: ROS message to be converted
    @return:
      - p: position as a np.array
      - q: quaternion as a numpy array (order = [x,y,z,w])
    """
    p = np.array([msg.translation.x, msg.translation.y, msg.translation.z])
    q = np.array([msg.rotation.x, msg.rotation.y,
                  msg.rotation.z, msg.rotation.w])
    return p, q


def transform_stamped_to_pq(msg):
    """Convert a C{geometry_msgs/TransformStamped} into position/quaternion np arrays

    @param msg: ROS message to be converted
    @return:
      - p: position as a np.array
      - q: quaternion as a numpy array (order = [x,y,z,w])
    """
    return transform_to_pq(msg.transform)


def msg_to_se3(msg):
    """Conversion from geometric ROS messages into SE(3)

    @param msg: Message to transform. Acceptable types - C{geometry_msgs/Pose}, C{geometry_msgs/PoseStamped},
    C{geometry_msgs/Transform}, or C{geometry_msgs/TransformStamped}
    @return: a 4x4 SE(3) matrix as a numpy array
    @note: Throws TypeError if we receive an incorrect type.
    """
    if isinstance(msg, Pose):
        p, q = pose_to_pq(msg)
    elif isinstance(msg, PoseStamped):
        p, q = pose_stamped_to_pq(msg)
    elif isinstance(msg, Transform):
        p, q = transform_to_pq(msg)
    elif isinstance(msg, TransformStamped):
        p, q = transform_stamped_to_pq(msg)
    else:
        raise TypeError("Invalid type for conversion to SE(3)")
    norm = np.linalg.norm(q)
    if np.abs(norm - 1.0) > 1e-3:
        raise ValueError(
            "Received un-normalized quaternion (q = {0:s} ||q|| = {1:3.6f})".format(
                str(q), np.linalg.norm(q)))
    elif np.abs(norm - 1.0) > 1e-6:
        q = q / norm
    g = tr.quaternion_matrix(q)
    g[0:3, -1] = p
    return g

class Collect():
    def __init__(self):
        self.bridge = CvBridge()

        # Mssage filter
        self.color_right = message_filters.Subscriber('/camera_mid/color/image_raw', Image)
        self.depth_right = message_filters.Subscriber('/camera_mid/aligned_depth_to_color/image_raw', Image)

        ts = message_filters.TimeSynchronizer([self.color_right, self.depth_right], 10)
        ts.registerCallback(self.callback_msgs)
        self.listener = tf.TransformListener()
        self.num = 0
        self.lock = True
        self.ee_pose = Pose()
        print("start")

    def callback_msgs(self, colorR, depthR):

        self.color_right = colorR
        self.depth_right = depthR
        cv_image = self.bridge.imgmsg_to_cv2(self.color_right, "bgr8")
        cv_depth = self.bridge.imgmsg_to_cv2(self.depth_right, "16UC1")

        cv2.imwrite('/home/arg-vx300s/Pick-and-Place-with-RL/d435/rgb/'+str(self.num)+'.png', cv_image)
        cv2.imwrite('/home/arg-vx300s/Pick-and-Place-with-RL/d435/depth/'+str(self.num)+'.png', cv_depth)
        print('/home/arg-vx300s/Pick-and-Place-with-RL/d435/rgb/'+str(self.num)+'.png')
        while self.lock == True:
            try:
                (trans,rot) = self.listener.lookupTransform('/left_arm/base_link', '/left_arm/ee_arm_link', rospy.Time(0))
                self.lock = False
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                self.lock = True
                continue
        self.ee_pose.position.x = trans[0]
        self.ee_pose.position.y = trans[1]
        self.ee_pose.position.z = trans[2]

        self.ee_pose.orientation.x = rot[0]
        self.ee_pose.orientation.y = rot[1]
        self.ee_pose.orientation.z = rot[2]
        self.ee_pose.orientation.w = rot[3]
        matrix = msg_to_se3(self.ee_pose)
        matrix = matrix.reshape(1,16)
        matrix = matrix.tolist()
        # print(matrix)
        data = {
            'EE_pose':[{
                'cols':4,
                'transformation_matrix': matrix,
                'rows':4}
            ]
        }
        self.lock = True
        with open('/home/arg-vx300s/Pick-and-Place-with-RL/d435/ee_pose/'+str(self.num)+'.yaml', 'w') as outfile:
            yaml.dump(data, outfile, default_flow_style=None)

        self.num += 1
        time.sleep(0.3)

if __name__ == '__main__':
    rospy.init_node("collect", anonymous=False)
    collect = Collect()
    rospy.spin()