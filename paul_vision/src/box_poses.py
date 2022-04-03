#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
from paul_vision.msg import BBox2d, BBox2d_array, BBox3d, BBox3d_array
import sys
import os
import numpy as np
import pyrealsense2 as rs2


class transform_pose:
    def imageDepthCallback(self, data):
        if self.input_bbox_array is not None:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
                if self.intrinsics:
                    bbox_3d_array = BBox3d_array()
                    for box in self.input_bbox_array:
                        bounding_box_3d = BBox3d()

                        center = [box.x1 + (box.x2-box.x1)/2, box.y1 + (box.y2-box.y1)/2]
                        result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [center[0], center[1]], cv_image[int(center[1]), int(center[0])])
                        bounding_box_3d.depth = result[2] / 1000

                        result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [box.x1, box.y1], cv_image[int(box.x1), int(box.y1)])
                        bounding_box_3d.x1 = result[0] / 1000
                        bounding_box_3d.y1 = result[1] / 1000

                        result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [box.x2, box.y2], cv_image[int(box.x2), int(box.y2)])
                        bounding_box_3d.x2 = result[0] / 1000
                        bounding_box_3d.y2 = result[1] / 1000
                        
                        bbox_3d_array.append(bounding_box_3d)
                    bbox_3d_array.header = data.header
                    self.pose_pub.publish(bbox_3d_array)
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
        self.input_bbox_array = bbox_array.boxes



    def __init__(self, depth_image_topic, depth_info_topic, bounding_box_topic):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(depth_image_topic, msg_Image, self.imageDepthCallback)
        self.sub_info = rospy.Subscriber(depth_info_topic, CameraInfo, self.imageDepthInfoCallback) 
        self.sub_bbox = rospy.Subscriber(bbox_segmentation_topic, BBox2d_array, self.BBoxCallback) 
        self.pose_pub = rospy.Publisher('bounding_boxes_3d', BBox3d_array, queue_size = 1) 
        

        self.intrinsics = None
        self.input_bbox_array = None



def main():
    depth_image_topic = '/camera/aligned_depth_to_color/image_raw'
    depth_info_topic = '/camera/aligned_depth_to_color/camera_info'
    bbox_segmentation_topic = '/bounding_boxes_segmentation'

    
    node = transform_pose(depth_image_topic, depth_info_topic, bounding_box_topic)
    rospy.spin()

if __name__ == '__main__':
    node_name = os.path.basename(sys.argv[0]).split('.')[0]
    rospy.init_node(node_name)
    main()