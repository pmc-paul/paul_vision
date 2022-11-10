#!/usr/bin/env python3
import numpy as np
import cv2
import matplotlib.pyplot as plt
import rospy
import os
import rospkg

from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String, Int64, Bool
from paul_vision.msg import BBox2d, BBox2d_array, item, classified_items
from geometry_msgs.msg import Pose, PoseArray, PointStamped
from sensor_msgs.msg import Image
bridge = CvBridge()


class master_vision:
    def __init__(self):
        self.item_found = False
        self.item_wanted = ''
        self.class_over = False
        self.items_list = classified_items()

    def classification_task_callback(self, msg):
        if not msg.data:
            self.classification_pub.publish(True)
        else:
            self.classification_pub.publish(False)
            for article in self.items_list.items:
                if article.name == self.item_wanted:
                    print('found')
                    # orientation camera (z,-x, -y) = orientation bras (x,y,z)
                    pose_pub = Pose()
                    pose_pub.position.x = article.box_3d.depth -0.027060000225901604
                    pose_pub.position.y = -article.box_3d.centerx + 0.009970000013709068
                    pose_pub.position.z = -article.box_3d.centery + 0.004705999977886677
                    self.arm_pub.publish(pose_pub)
                    # x = -0.027060000225901604 ; y = -0.009970000013709068 ; z = -0.004705999977886677
    
    def resquest_callback(self, request):
        self.classification_pub.publish(True)
        self.item_wanted = request.data


    def classified_articles_callback(self, articles_list):
        self.items_list = articles_list


    def main(self):
        rospy.init_node('master_vision')
        rospy.Subscriber('/classification_finished', Bool, self.classification_task_callback)
        rospy.Subscriber('/item_request', String, self.resquest_callback)
        rospy.Subscriber('/classified_items', classified_items, self.classified_articles_callback)
        self.arm_pub = rospy.Publisher('/arm_position_request', Pose, queue_size=1)
        self.classification_pub = rospy.Publisher('/classification_search', Bool, queue_size=1)
        rospy.spin() 


if __name__ == '__main__':
    node = master_vision()
    node.main()
