#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose, PointStamped
from cv_bridge import CvBridge, CvBridgeError
from paul_vision.msg import BBox2d, BBox2d_array, BBox3d, BBox3d_array, article, classified_items

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
    def init(self, bounding_box_topic):
        # coordonnees avec rail
        self.original_arm_pose = Pose()
        self.original_arm_pose.Point.x = 0
        self.original_arm_pose.Point.y = 0
        self.original_arm_pose.Point.z = 0

        self.current_arm_pose = Pose()
        self.current_arm_pose.Point.x = 0
        self.current_arm_pose.Point.y = 0
        self.current_arm_pose.Point.z = 0.4 # meters

        self.bridge = CvBridge()
        # self.sub = rospy.Subscriber(depth_image_topic, msg_Image, self.imageDepthCallback)
        # self.sub_info = rospy.Subscriber(depth_info_topic, CameraInfo, self.imageDepthInfoCallback) 
        self.sub_bbox = rospy.Subscriber(bounding_box_topic, BBox3d_array, self.BBoxCallback) 


    def arm_go_to(self, boxes):
        print(boxes)
        # calculate arm coordonates for batches?
        # left to right

    def BBoxCallback(self, msg):
        for box in msg:
            print(box)

#  mode sans segmentation
class one_step_classification:

    # callback pour chaque match + 25 points
    # comparaison iou pour bounding box
    # assignement d'un id si nouvelle boite
    # comparaison confidence si pas nouveau
    # changement nom et confidence si nouveau match meilleur



    def get_iou(bb1, bb2):
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
        # assert bb1['x1'] < bb1['x2']
        # assert bb1['y1'] < bb1['y2']
        # assert bb2['x1'] < bb2['x2']
        # assert bb2['y1'] < bb2['y2']

        # determine the coordinates of the intersection rectangle
        x_left = max(bb1['x1'], bb2['x1'])
        y_top = max(bb1['y1'], bb2['y1'])
        x_right = min(bb1['x2'], bb2['x2'])
        y_bottom = min(bb1['y2'], bb2['y2'])

        if x_right < x_left or y_bottom < y_top:
            return 0.0

        # The intersection of two axis-aligned bounding boxes is always an
        # axis-aligned bounding box
        intersection_area = (x_right - x_left) * (y_bottom - y_top)

        # compute the area of both AABBs
        bb1_area = (bb1['x2'] - bb1['x1']) * (bb1['y2'] - bb1['y1'])
        bb2_area = (bb2['x2'] - bb2['x1']) * (bb2['y2'] - bb2['y1'])

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

    
    node = two_step_classification(bounding_box_topic)
    # node = one_step_classification()
    rospy.spin()

if __name__ == '__main__':
    node_name = 'vision_database'
    rospy.init_node(node_name)
    main()