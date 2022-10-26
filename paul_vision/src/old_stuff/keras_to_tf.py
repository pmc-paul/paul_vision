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
model.save('/home/victoria/paul/dev_ws/src/paul_vision/paul_vision/model.pb')