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
        self.bounding_boxes = None

    def image_callback(self, camera_stream):
         # to use without the segmentation pre-process
        try: 
            image = bridge.imgmsg_to_cv2(camera_stream, 'bgr8')
        except CvBridgeError as e:
            print("CvBridge could not convert images from realsense to opencv")
        if self.bounding_boxes is not None:
            # print(self.bounding_boxes.boxes)
            for bounding_box in self.bounding_boxes.boxes:
                cv2.rectangle(image, (int(bounding_box.x1), int(bounding_box.y1)), (int(bounding_box.x2), int(bounding_box.y2)), (0,0,255), 2)
            image_pub = bridge.cv2_to_imgmsg(image)
            self.pub_image.publish(image_pub)


    def items_callback(self, bbox_msg):
        self.bounding_boxes = BBox2d_array()
        for item in bbox_msg.items:
            self.bounding_boxes.boxes.append(item.box_2d)

def main():
    node = vizualisation()
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node('viz')
    main()