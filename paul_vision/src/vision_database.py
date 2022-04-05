#!/usr/bin/env python3
import rospy
import sys
import os
import numpy as np
from geometry_msgs.msg import Pose, PointStamped
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image
from realsense2_camera.msg import Extrinsics
from paul_vision.msg import BBox2d, BBox2d_array, BBox3d, BBox3d_array, item, classified_items
import pyrealsense2 as rs2
import cv2

## bounding boxes complete info:
# bounding box 3d from original
# original arm pose
# current arm pose
# current 2d pose in video feed
# number id
# class (food)
# confidence (number of features matched)

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

    # callback pour chaque match + 35 points
    # comparaison iou pour bounding box
    # assignement d'un id si nouvelle boite
    # comparaison confidence si pas nouveau
    # changement nom et confidence si nouveau match meilleur

    def __init__(self):
        rospy.Subscriber('/new_item', item, self.new_item_callback)
        self.id = 0
        self.classified_items = classified_items()


    def new_item_callback(self, new_item):
        print('new item!')
        if len(self.classified_items.items)>0:
            match = None
            for article in self.classified_items.items:
                iou = self.get_iou(new_item.box_2d, article.box_2d)
                # print(str(iou) +' ' + article.name + new_item.name)
                if iou > 0.70:
                    match = article.item_id
                    if new_item.confidence > article.confidence:
                        article.name = new_item.name
                        article.confidence = new_item.confidence
                        article.box_2d = new_item.box_2d
            if match is None:
                # print('no match')
                new_item.item_id = self.id
                self.id += 1
                self.classified_items.items.append(new_item)
        else:
            new_item.item_id = self.id
            self.id += 1
            self.classified_items.items.append(new_item)
        print(self.classified_items)



    def get_iou(self, bb1, bb2):
        """
        Calculate the Intersection over Union (IoU) of two bounding boxes.

        Parameters
        ----------
        bb1 : dict
            Keys: {'x1', 'x2', 'y1', 'y2'}
            The (x1, y1) position is at the top left corner,
            the (x2, y2) position is at the bottom right corner
        bb2 : dict
            Keys: {'x1', 'x2', 'y1', 'y2'}
            The (x, y) position is at the top left corner,
            the (x2, y2) position is at the bottom right corner

        Returns
        -------
        float
            in [0, 1]
        """
        # assert bb1.x1 < bb1.x2
        # assert bb1.y1 < bb1.y2
        # assert bb2.x1 < bb2.x2
        # assert bb2.y1 < bb2.y2

        # determine the coordinates of the intersection rectangle
        x_left = max(bb1.x1, bb2.x1)
        y_top = max(bb1.y1, bb2.y1)
        x_right = min(bb1.x2, bb2.x2)
        y_bottom = min(bb1.y2, bb2.y2)

        if x_right < x_left or y_bottom < y_top:
            return 0.0

        # The intersection of two axis-aligned bounding boxes is always an
        # axis-aligned bounding box
        intersection_area = (x_right - x_left) * (y_bottom - y_top)

        # compute the area of both AABBs
        bb1_area = (bb1.x2 - bb1.x1) * (bb1.y2 - bb1.y1)
        bb2_area = (bb2.x2 - bb2.x1) * (bb2.y2 - bb2.y1)

        # compute the intersection over union by taking the intersection
        # area and dividing it by the sum of prediction + ground-truth
        # areas - the interesection area
        iou = intersection_area / float(bb1_area + bb2_area - intersection_area)
        # assert iou >= 0.0
        # assert iou <= 1.0
        return iou

def main():
    depth_image_topic = '/camera/aligned_depth_to_color/image_raw'
    depth_info_topic = '/camera/aligned_depth_to_color/camera_info'
    bbox_3d_topic = '/bounding_boxes_3d'
    # bbox_classification_topic = 

    
    # node = two_step_classification()
    node = one_step_classification()
    rospy.spin()

if __name__ == '__main__':
    node_name = 'vision_database'
    rospy.init_node(node_name)
    main()