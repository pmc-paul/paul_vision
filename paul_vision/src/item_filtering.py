#!/usr/bin/env python3
import rospy
import sys
import os
import numpy as np
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Pose, PoseArray, PointStamped
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image
from realsense2_camera.msg import Extrinsics
from paul_vision.msg import BBox2d, BBox2d_array, item, classified_items
from paul_vision.msg import *
from paul_vision.srv import *
import pyrealsense2 as rs2
import cv2

# mode avec segmentation
class two_step_classification:
    def __init__(self):
        # coordonnees avec rail
        self.original_arm_pose = Pose()
        self.original_arm_pose.position.x = 0
        self.original_arm_pose.position.y = 0
        self.original_arm_pose.position.z = 0

        self.current_arm_pose = Pose()
        self.current_arm_pose.position.x = 0
        self.current_arm_pose.position.y = 0
        self.current_arm_pose.position.z = 0.61 # meters
        self.extrinsics_depth = None
        self.intrinsics_color = None
        self.color_pixel_up = None
        self.color_pixel_down = None
        self.bbox = None
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber('/camera/color/image_raw', Image, self.imageColorCallback)
        self.sub_info = rospy.Subscriber('/camera/extrinsics/depth_to_color', Extrinsics, self.imageDepthInfoCallback) 
        self.sub_info = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.imageColorInfoCallback) 
        self.sub_bbox = rospy.Subscriber('/bounding_boxes_3d', BBox3d_array, self.BBoxCallback)
        self.pub_image = rospy.Publisher('prediction_2d_bbox', Image, queue_size = 1)
        self.pub_bbox2d = rospy.Publisher('classification_bounding_boxes', BBox2d_array, queue_size = 1)
        print('init') 


    def arm_go_to(self, boxes):
        print(boxes)
        # calculate arm coordonates for batches?
        # left to right


    def imageDepthInfoCallback(self, extrinsics_msg):
        self.extrinsics_depth = rs2.extrinsics()
        self.extrinsics_depth.rotation = extrinsics_msg.rotation
        self.extrinsics_depth.translation = extrinsics_msg.translation
         
    def imageColorInfoCallback(self, cameraInfo):
        try:
            if self.intrinsics_color:
                return
            self.intrinsics_color = rs2.intrinsics()
            self.intrinsics_color.width = cameraInfo.width
            self.intrinsics_color.height = cameraInfo.height
            self.intrinsics_color.ppx = cameraInfo.K[2]
            self.intrinsics_color.ppy = cameraInfo.K[5]
            self.intrinsics_color.fx = cameraInfo.K[0]
            self.intrinsics_color.fy = cameraInfo.K[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics_color.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics_color.model = rs2.distortion.kannala_brandt4
            self.intrinsics_color.coeffs = [i for i in cameraInfo.D]
        except CvBridgeError as e:
            print(e)
            return

    def BBoxCallback(self, msg):
        # for box in msg.boxes:
        box = msg.boxes[0]
        box.depth = box.depth - (self.current_arm_pose.position.z - self.original_arm_pose.position.z)
        # box.depth = 0.5
        depth_point = [box.x1, box.y1, box.depth]
        # print(depth_point)
        # remettre point to pint pour tester de proche
        # color_point = rs2.rs2_transform_point_to_point(self.extrinsics_depth, depth_point)
        self.color_pixel_up = rs2.rs2_project_point_to_pixel(self.intrinsics_color, depth_point)
        depth_point = [box.x2, box.y2, box.depth]
        # color_point = rs2.rs2_transform_point_to_point(self.extrinsics_depth, depth_point)
        self.color_pixel_down = rs2.rs2_project_point_to_pixel(self.intrinsics_color, depth_point)
        print(depth_point)
        print(self.color_pixel_down)
        bbox_msg = BBox2d_array()
        self.bbox = BBox2d()
        self.bbox.x1 = (self.color_pixel_up[1])
        self.bbox.y1 = (self.color_pixel_up[0])
        self.bbox.x2 = (self.color_pixel_down[1])
        self.bbox.y2 = (self.color_pixel_down[0])
        # self.bbox.x1 = 0.8*(640-self.color_pixel_up[0])
        # self.bbox.y1 = 0.8*(480-self.color_pixel_up[1])
        # self.bbox.x2 = 1.2*(640-self.color_pixel_down[0])
        # self.bbox.y2 = 1.2*(480-self.color_pixel_down[1])
        bbox_msg.boxes.append(self.bbox)
        self.pub_bbox2d.publish(bbox_msg)
        
        
    def imageColorCallback(self, msg):
        if self.bbox is not None:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            color = (0, 0, 255)
            bbox_image = cv_image.copy()
            cv2.rectangle(bbox_image, (int(self.bbox.x1), int(self.bbox.y1)), (int(self.bbox.x2), int(self.bbox.y2)), color, 2)
            # cv2.rectangle(bbox_image, (int(550), int(278)), (int(800), int(410)), color, 2)
            print(self.bbox)
            # print(self.color_pixel_down[0])
            # print(int(self.color_pixel_up[0]))
            image_pub = self.bridge.cv2_to_imgmsg(bbox_image)
            self.pub_image.publish((image_pub))
            

#  mode sans segmentation
class one_step_classification:
    def __init__(self):
        # self.id = 0
        self.filtering = False
        self.classified_items = classified_items()
        self.request_sent = False
        rospy.Subscriber('/new_item', item, self.new_item_callback)
        self.classified_pub = rospy.Publisher('/classified_items', classified_items, queue_size=1)
        self.pose_array_pub = rospy.Publisher('/pose_array', PoseArray, queue_size=1)
        rospy.Subscriber('/classification_finished', Bool, self.classification_callback)


    def new_item_callback(self, new_item):
        self.filtering = True
        if len(self.classified_items.items)>0:
            match = None
            for article in self.classified_items.items:
                # changer nms pour traiter boite par ordre de confiance
                iou = self.get_iou(new_item.box_2d, article.box_2d)
                # print(str(iou) +' ' + article.name + new_item.name)
                if iou > 0.50:
                    match = article.item_id
                    if new_item.confidence > article.confidence:
                        article.name = new_item.name
                        article.item_id = new_item.item_id
                        article.confidence = new_item.confidence
                        article.box_2d = new_item.box_2d
            if match is None: # and len(self.classified_items.items)<4:
                # # print('no match')
                # new_item.item_id = self.id
                # self.id += 1
                self.classified_items.items.append(new_item)
        elif len(self.classified_items.items) == 0:
            # new_item.item_id = self.id
            # self.id += 1
            self.classified_items.items.append(new_item)
        for article in self.classified_items.items:
            # call service
            rospy.wait_for_service('change_2d_to_3d')
            try:
                transform = rospy.ServiceProxy('change_2d_to_3d', change_2d_to_3d)
                bounding_box_3d = BBox3d()
                bounding_box_3d = transform(article.box_2d).box_3d
                article.box_3d = bounding_box_3d
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
        if len(self.classified_items.items) > 0:
            point_pub = PoseArray()
            for articles in self.classified_items.items:
                points = Pose()
                points.position.x = articles.box_3d.centerx
                points.position.y = articles.box_3d.centery
                points.position.z = articles.box_3d.depth 
                point_pub.poses.append(points)
                point_pub.header = articles.header
            self.pose_array_pub.publish(point_pub)
        self.filtering = False

    def classification_callback(self, msg):
        print("found " + str(len(self.classified_items.items)) + " items")
        while(self.filtering):
            rospy.sleep(0.1)
        if len(self.classified_items.items) > 0:
            self.classified_pub.publish(self.classified_items)

    def get_iou(self, bb1, bb2):

        x_left = max(bb1.x1, bb2.x1)
        y_top = max(bb1.y1, bb2.y1)
        x_right = min(bb1.x2, bb2.x2)
        y_bottom = min(bb1.y2, bb2.y2)

        if x_right < x_left or y_bottom < y_top:
            return 0.0

        intersection_area = (x_right - x_left) * (y_bottom - y_top)

        bb1_area = (bb1.x2 - bb1.x1) * (bb1.y2 - bb1.y1)
        bb2_area = (bb2.x2 - bb2.x1) * (bb2.y2 - bb2.y1)

        iou = intersection_area / float(bb1_area + bb2_area - intersection_area)
        return iou

def main():
    node = one_step_classification()
    rospy.spin()

if __name__ == '__main__':
    node_name = 'vision_database'
    rospy.init_node(node_name)
    main()