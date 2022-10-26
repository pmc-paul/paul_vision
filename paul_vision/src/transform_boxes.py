#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
from paul_vision.msg import BBox2d, BBox2d_array, BBox3d, BBox3d_array
from paul_vision.srv import change_2d_to_3d, change_2d_to_3dResponse
import sys
import os
import numpy as np
import pyrealsense2 as rs2

class transform:
    def handle_2d_to_3d(self, boxinput):
        box = boxinput.box_2d
        bounding_box_3d = BBox3d()
        if self.intrinsics is not None and self.cv_image is not None:
            center = [box.y1 + (box.y2-box.y1)/2, box.x1 + (box.x2-box.x1)/2]
            image_copy = self.cv_image.copy()
            result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [center[1], center[0]], image_copy[int(center[0]), int(center[1])])
            bounding_box_3d.centery = result[1] / 1000
            bounding_box_3d.centerx = result[0] / 1000
            bounding_box_3d.depth = result[2] / 1000

            result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [box.x1, box.y1], image_copy[int(center[0]), int(center[1])])
            bounding_box_3d.y1 = result[1] / 1000
            bounding_box_3d.x1 = result[0] / 1000

            result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [box.x2, box.y2], image_copy[int(center[0]), int(center[1])])
            bounding_box_3d.y2 = result[1] / 1000
            bounding_box_3d.x2 = result[0] / 1000

        return change_2d_to_3dResponse(bounding_box_3d)


    def imageDepthCallback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding).copy()
        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            print(e)
            return
    # d435 intrinsics?????
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

    def __init__(self):
        self.intrinsics = None
        self.bridge = CvBridge()
        self.cv_image = None

        s = rospy.Service('change_2d_to_3d', change_2d_to_3d, self.handle_2d_to_3d)
        sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', msg_Image, self.imageDepthCallback)
        sub_info = rospy.Subscriber('/camera/aligned_depth_to_color/camera_info', CameraInfo, self.imageDepthInfoCallback)


def transform_server():
    node = transform() 
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node('transform_boxes_server')
    transform_server()