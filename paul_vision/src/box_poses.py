#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
from paul_vision.msg import BBox2d, BBox2d_array
import sys
import os
import numpy as np
import pyrealsense2 as rs2


class PoseExport:

    def rescale_boxes(self, boxes, img_width=848, img_height=480, predict_width=1333, predict_height=800):
        scalex = img_width / predict_width
        scaley = img_height / predict_height
        for box in boxes:
            box.x1 = box.x1 * scalex
            box.y1 = box.y1 * scaley
            box.x2 = box.x2 * scalex
            box.y2 = box.y2 * scaley
        return boxes


    # def convert_phys_to_pixel_coord_using_realsense(self, x, y, cameraInfo):
    #     _intrinsics = rs2.intrinsics()
    #     _intrinsics.width = cameraInfo.width
    #     _intrinsics.height = cameraInfo.height
    #     _intrinsics.ppx = cameraInfo.K[2]
    #     _intrinsics.ppy = cameraInfo.K[5]
    #     _intrinsics.fx = cameraInfo.K[0]
    #     _intrinsics.fy = cameraInfo.K[4]
    #     #_intrinsics.model = cameraInfo.distortion_model
    #     _intrinsics.model  = rs2.distortion.none
    #     _intrinsics.coeffs = [i for i in cameraInfo.D]
    #     result = rs2.rs2_point_to_pixel(_intrinsics, [x, y])
    #     #result[0]: right, result[1]: down, result[2]: forward
    #     return result[2], -result[0], -result[1]
  
    def imageDepthCallback(self, data):
        if self.pix is not None:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
                # line = '\rDepth at pixel(%3d, %3d): %7.1f(mm).' % (self.pix[0], self.pix[1], cv_image[self.pix[1], self.pix[0]])
                if self.intrinsics:
                    point = PointStamped()
                    depth = cv_image[int(self.pix[1]), int(self.pix[0])]
                    # print(self.pix)
                    result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [self.pix[0], self.pix[1]], depth)
                    point.point.x = result[0] / 1000
                    point.point.y = result[1] / 1000
                    point.point.z = result[2] / 1000
                    point.header = data.header
                    self.pose_pub.publish(point)
            except CvBridgeError as e:
                print(e)
                return
            except ValueError as e:
                print(e)
                return

    def imageDepthInfoCallback(self, cameraInfo):
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.K[2]
            self.intrinsics.ppy = cameraInfo.K[5]
            self.intrinsics.fx = cameraInfo.K[0]
            self.intrinsics.fy = cameraInfo.K[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.D]
        except CvBridgeError as e:
            print(e)
            return

    def BBoxCallback(self, bbox_array):
        self.array = bbox_array.boxes
        # self.array = self.rescale_boxes(temp_array)
        left_most = [self.array[0].x1,self.array[1].y1]
        for box in self.array:
            if (box.x1 + (box.x2-box.x1)) < left_most[0]:
                left_most[0] = box.x1 + (box.x2-box.x1)/2
                left_most[1] = box.y1 + (box.y2-box.y1)/2
        # pour erreur entre depth et color
        # depth_left_most = rs2.rs2_project_color_pixel_to_depth_pixel(left_most)
        self.pix = left_most
        # print(self.pix)


    def __init__(self, depth_image_topic, depth_info_topic, bounding_box_topic):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(depth_image_topic, msg_Image, self.imageDepthCallback)
        self.sub_info = rospy.Subscriber(depth_info_topic, CameraInfo, self.imageDepthInfoCallback) 
        self.sub_bbox = rospy.Subscriber(bounding_box_topic, BBox2d_array, self.BBoxCallback) 
        self.pose_pub = rospy.Publisher('arm_go_to', PointStamped, queue_size = 1) 

        self.intrinsics = None
        self.pix = None
        # self.pix_grade = None
        self.array = None

def main():
    depth_image_topic = '/camera/aligned_depth_to_color/image_raw'
    depth_info_topic = '/camera/aligned_depth_to_color/camera_info'
    bounding_box_topic = '/bounding_boxes'
    
    listener = PoseExport(depth_image_topic, depth_info_topic, bounding_box_topic)
    rospy.spin()

if __name__ == '__main__':
    node_name = os.path.basename(sys.argv[0]).split('.')[0]
    rospy.init_node(node_name)
    main()