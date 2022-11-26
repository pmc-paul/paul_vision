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
        center = None
        if self.d435:
            center = [box.y1 + (box.y2-box.y1)/2, box.x1 + (box.x2-box.x1)/2]
        else:
            if self.largeur3D is not None and self.largeur2D is not None :
                resolutionRatio = self.largeur3D/self.largeur2D
                center2dy = box.y1 + (box.y2-box.y1)/2
                center2dx = box.x1 + (box.x2-box.x1)/2
                # compare center x to image center for proportion
                diff_center = (self.largeur2D/2) - center2dx
                proportion_diff = (1 - abs(diff_center/self.largeur2D/2))
                if abs(diff_center)<100:
                    proportion_diff = proportion_diff/4
                elif abs(diff_center)<200:
                    proportion_diff = proportion_diff/2
                elif abs(diff_center)<350:
                    proportion_diff = proportion_diff/1.5
                if abs(diff_center) > 500:
                    proportion_diff = proportion_diff * 1.7
                side = diff_center / abs(diff_center)
                centery = (center2dy * resolutionRatio ) + (self.largeur2D * 0.03)
                centerx = (center2dx * resolutionRatio) + (side * proportion_diff * self.largeur2D * 0.03)
                center = [centery, centerx]

        if self.intrinsics is not None and self.cv_image is not None:
            image_copy = self.cv_image.copy()
            # if image_copy[int(center[0]), int(center[1])] == 0 :
            center = [centery, centerx]
            pixel_search = 16
            neighboor_depth = []
            sum_depth = 0
            for i in range(0, pixel_search):
                for j in range(0, pixel_search):
                    curr_depth = (image_copy[int(centery+i-(pixel_search/2)), int(centerx+j-(pixel_search/2))])
                    if curr_depth != 0:
                        neighboor_depth.append(curr_depth)
                        sum_depth += curr_depth
            if len(neighboor_depth)>0:
                mean_depth = sum_depth / len(neighboor_depth)
                depth = mean_depth
                
                result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [center[1], center[0]], depth)
                bounding_box_3d.centery = result[1] / 1000
                bounding_box_3d.centerx = result[0] / 1000
                bounding_box_3d.depth = result[2] / 1000

                result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [box.x1, box.y1], depth)
                bounding_box_3d.y1 = result[1] / 1000
                bounding_box_3d.x1 = result[0] / 1000

                result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [box.x2, box.y2], depth)
                bounding_box_3d.y2 = result[1] / 1000
                bounding_box_3d.x2 = result[0] / 1000
            
        if side < 0: # right, orientation inverted when shelf on the right of robot
            bounding_box_3d.centerx = bounding_box_3d.centerx
        # shelf facing robot cords
        bounding_box_3d.depth = -bounding_box_3d.depth
        return change_2d_to_3dResponse(bounding_box_3d)



    def image3dCallback(self, img):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(img, 'passthrough')
        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            print(e)
            return


    def info2dCallback(self, cameraInfo):
        try:
            if self.largeur2D:
                return
            self.largeur2D = cameraInfo.width
        except CvBridgeError as e:
            print(e)
            return

    def imageDepthCallback(self, data):
        try:
            # for d435
            self.cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding).copy()  
        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            print(e)
            return
    # d435 intrinsics
    def imageDepthInfoCallback(self, cameraInfo):
        try:
            if self.intrinsics:
                return
            if not self.d435:
                self.largeur3D = cameraInfo.width
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
        self.d435 = False
        self.largeur2D = None
        self.largeur3D = None
        s = rospy.Service('change_2d_to_3d', change_2d_to_3d, self.handle_2d_to_3d)
        # gen3 light + d435
        # sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', msg_Image, self.imageDepthCallback)
        # sub_info = rospy.Subscriber('/camera/aligned_depth_to_color/camera_info', CameraInfo, self.imageDepthInfoCallback)
        # gen3
        sub2d = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.info2dCallback)
        sub_image = rospy.Subscriber('/camera/depth/image_raw', msg_Image, self.image3dCallback)
        sub_info = rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.imageDepthInfoCallback)


def transform_server():
    node = transform() 
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node('transform_boxes_server')
    transform_server()