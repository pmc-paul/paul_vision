#!/usr/bin/env python3
import numpy as np
import cv2
import matplotlib.pyplot as plt
import rospy
import os
import rospkg

from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String, Int64, Bool
from paul_vision.msg import BBox2d, BBox2d_array, item
from geometry_msgs.msg import Pose, PoseArray, PointStamped
from sensor_msgs.msg import Image
bridge = CvBridge()


class master_vision:
    # def __init__(self):

    def classification_task_callback(self, msg):
        if msg:
            print(msg)

    def main():
        rospy.init_node('master_vision')
        rospy.Subscriber('/classification_bool', Bool, self.classification_task_callback)
        rospy.Subscriber('/item_request', String, self.resquest_callback)
        rospy.Subscriber('/classified_items', classified_items, self.classified_articles_callback)
        self.arm_pub = rospy.Publisher('/arm_position_request', Pose, queue_size=1)
        rospy.spin() 


if __name__ == '__main__':
    node = master_vision()
    node.main()
