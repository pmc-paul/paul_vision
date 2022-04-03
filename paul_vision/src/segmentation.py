#!/usr/bin/env python3
import rospy
import cv2
import roslib
import keras
import numpy as np
from paul_vision.msg import BBox2d, BBox2d_array
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import tensorflow as tf
import argparse
import sys
import os

from object_detector_retinanet.keras_retinanet import models
from object_detector_retinanet.utils import image_path, annotation_path, root_dir

import matplotlib.pyplot as plt

# import model and  implement fix found here.
# https://github.com/fchollet/keras/issues/2397
model = models.load_model(os.path.join(root_dir(), "iou_resnet50_csv_06.h5"), backbone_name='resnet50', convert=True, nms=True)
model._make_predict_function()

rospy.init_node('prediction_node')
boxes_pub = rospy.Publisher('bounding_boxes_segmentation', BBox2d_array, queue_size = 1)
pub_image = rospy.Publisher('prediction_image', Image, queue_size = 1)
bridge = CvBridge()

def resize_image(img, min_side=800, max_side=1333):
    """ Resize an image such that the size is constrained to min_side and max_side.

    Args
        min_side: The image's min side will be equal to min_side after resizing.
        max_side: If after resizing the image's max side is above max_side, resize until the max side is equal to max_side.

    Returns
        A resized image.
    """
    (rows, cols, _) = img.shape
    smallest_side = min(rows, cols)
    # rescale the image so the smallest side is min_side
    scaley = min_side / smallest_side

    # check if the largest side is now greater than max_side, which can happen
    # when images have a large aspect ratio
    largest_side = max(rows, cols)
    # if largest_side * scaley > max_side:
    scalex = max_side / largest_side

    # resize the image with the computed scale
    img = cv2.resize(img, None, fx=scalex, fy=scaley)
    return img, scalex, scaley

def preprocess_image(x):
    """ Preprocess an image by subtracting the ImageNet mean.

    Args
        x: np.array of shape (None, None, 3) or (3, None, None).

    Returns
        The input with the ImageNet mean subtracted.
    """
    # mostly identical to "https://github.com/fchollet/keras/blob/master/keras/applications/imagenet_utils.py"
    # except for converting RGB -> BGR since we assume BGR already
    x = x.astype(keras.backend.floatx())
    if keras.backend.image_data_format() == 'channels_first':
        if x.ndim == 3:
            x[0, :, :] -= 103.939
            x[1, :, :] -= 116.779
            x[2, :, :] -= 123.68
        else:
            x[:, 0, :, :] -= 103.939
            x[:, 1, :, :] -= 116.779
            x[:, 2, :, :] -= 123.68
    else:
        x[..., 0] -= 103.939
        x[..., 1] -= 116.779
        x[..., 2] -= 123.68

    return x

def callback(image_msg):
    #First convert the image to OpenCV image 
    original_img = bridge.imgmsg_to_cv2(image_msg, 'bgr8').copy()
    cv_image = bridge.imgmsg_to_cv2(image_msg, 'bgr8')
    cv_image, scalex, scaley = resize_image(cv_image)
    
    image = preprocess_image(cv_image.copy())
    boxes, hard_scores, labels, soft_scores = model.predict(np.expand_dims(image, axis=0))            # Classify the image
    soft_scores = np.squeeze(soft_scores, axis=-1)
    soft_scores = 0.5 * hard_scores + (1 - 0.5) * soft_scores

    # select indices which have a score above the threshold
    indices = np.where(hard_scores[0, :] > 0.6)[0]

    # select those scores
    scores = soft_scores[0][indices]
    hard_scores = hard_scores[0][indices]

    # find the order with which to sort the scores
    scores_sort = np.argsort(-scores)[:10] #max detections

    # select detections
    image_boxes = boxes[0, indices[scores_sort], :]
    image_scores = scores[scores_sort]
    image_hard_scores = hard_scores[scores_sort]

    color = (0, 0, 255)
    
    boxes_msg = BBox2d_array()
    # print(image_boxes[:,0])
    bbox_image = cv_image.copy()
    for detection in image_boxes:
        bounding_box = BBox2d()
        bounding_box.x1 = detection[0] / scalex
        bounding_box.y1 = detection[1] / scaley
        bounding_box.x2 = detection[2] / scalex
        bounding_box.y2 = detection[3] / scaley
        boxes_msg.boxes.append(bounding_box)
        cv2.rectangle(original_img, (int(bounding_box.x1), int(bounding_box.y1)), (int(bounding_box.x2), int(bounding_box.y2)), color, 2)
    if len(boxes_msg.boxes):
        print(boxes_msg.boxes)
        # try:
        #     boxes_msg.image = bridge.cv2_to_imgmsg(cv_image, 'bgr8')
        # except CvBridgeError as e:
        #     print(e)
        boxes_pub.publish(boxes_msg)
    image_pub = bridge.cv2_to_imgmsg(original_img)
    pub_image.publish((image_pub))
    # plt.imshow(cv_image),plt.show()

        

rospy.Subscriber("camera/color/image_raw", Image, callback, queue_size = 1, buff_size = 16777216)



while not rospy.is_shutdown():
  rospy.spin()