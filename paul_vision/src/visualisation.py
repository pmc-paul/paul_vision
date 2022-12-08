#!/usr/bin/env python3
import numpy as np
import cv2
import matplotlib.pyplot as plt
import rospy
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String, Int64
from paul_vision.msg import BBox2d, BBox2d_array, BBox3d, BBox3d_array, classified_items
from sensor_msgs.msg import Image
import os
bridge = CvBridge()
import rospkg

class vizualisation:
    def __init__(self):
        rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        rospy.Subscriber('/classified_items', classified_items, self.items_callback)
        self.pub_image = rospy.Publisher('/classified_image', Image, queue_size=2)
        # self.items_list = None
        self.image = None

    def image_callback(self, camera_stream):
         # to use without the segmentation pre-process
        try: 
            self.image = bridge.imgmsg_to_cv2(camera_stream, 'bgr8')
        except CvBridgeError as e:
            print("CvBridge could not convert images from realsense to opencv")


    def items_callback(self, bbox_msg):
        self.items_list = BBox2d_array()
        if self.image is not None:
            image_copy = self.image.copy()
            for item in bbox_msg.items:
                # self.items_list.boxes.append(item)
                bounding_box = item.box_2d
                color = (0,0,255)
                if item.name == 'champ_leger.png':
                    color = (255,0,50)
                if item.name == 'champ_sans_sel.png':
                    color = (255,255,0)
                cv2.putText(image_copy, item.name, (int(bounding_box.x1), int(bounding_box.y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                cv2.rectangle(image_copy, (int(bounding_box.x1), int(bounding_box.y1)), (int(bounding_box.x2), int(bounding_box.y2)), color, 2)
                image_pub = bridge.cv2_to_imgmsg(image_copy)
                self.pub_image.publish(image_pub)

def main():
    node = vizualisation()
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node('viz')
    main()