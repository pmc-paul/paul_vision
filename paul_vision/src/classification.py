#!/usr/bin/env python3
import numpy as np
import cv2
import matplotlib.pyplot as plt
import rospy
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String, Int64, Bool
from paul_vision.msg import BBox2d, BBox2d_array, item, classified_items
from sensor_msgs.msg import Image
import os
bridge = CvBridge()
import rospkg
from database.srv import *


class find_item:
    def __init__(self):
        self.article_path = ''
        self.bbox_array = []
        self.iterations = 0
        self.camera_stream = None
        self.pixelRation = 35
        self.pixelRange = self.pixelRation * 2.15

    def image_callback_matching(self, image):
        # to use without the segmentation pre-process
        try: 
            self.camera_stream = bridge.imgmsg_to_cv2(image, 'bgr8')
            self.header = image.header
        except CvBridgeError as e:
            print("CvBridge could not convert images from realsense to opencv")
        
    def nameRequest(self, name):
        rospy.wait_for_service('sqlRequestName')
        try:
            db_request = rospy.ServiceProxy('sqlRequestName', itemDetailsName)
            resp1 = db_request(str(name))
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        return resp1


    def processing_callback(self, msg):
        print('new request for: ' + str(msg.data))
        if self.camera_stream is not None:
            iteration = 0
            while iteration < 2:
                # sections = [[0,640], [240,900], [640,1280]]
                # for section in sections:
                # img2 = stream[0:,section[0]:section[1]].copy()
                # decision arbitraire à revoir
                minimum_match = 40
                article_match = ''
                img2 = self.camera_stream.copy()
                img2 = cv2.resize(img2,(img2.shape[1],img2.shape[0]))
                img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

                sift = cv2.SIFT_create()
                keypoints2, descriptors2 = sift.detectAndCompute(img2, None)
                level =  self.nameRequest(msg.data + ".png").level
                if level != 0:
                    cwd = rospkg.RosPack().get_path('paul_vision')
                    articles_folder = cwd + '/level' + str(int(level))+ '/'
                    for article in os.listdir(articles_folder):
                        article_path = articles_folder + article
                        img1 = cv2.imread(article_path,cv2.IMREAD_GRAYSCALE) # queryImage
                        img1 = cv2.normalize(img1, None, 0, 255, cv2.NORM_MINMAX)
                        
                        # match stream and reference
                        keypoints1, descriptors1 = sift.detectAndCompute(img1, None)
                        FLAN_INDEX_KDTREE = 0
                        index_params = dict (algorithm = FLAN_INDEX_KDTREE, trees=5)
                        search_params = dict (checks=50)
                        flann = cv2.FlannBasedMatcher(index_params, search_params)
                        matches = flann.knnMatch (descriptors1, descriptors2, k=2)
                        matchesMask = [[0,0] for i in range(len(matches))]
                        for i,(m1, m2) in enumerate (matches):
                            if m1.distance < 0.5 * m2.distance:
                                matchesMask[i] = [1,0]

                        # Sort by their distance.
                        matches = sorted(matches, key = lambda x:x[0].distance)
                        good = [m1 for (m1, m2) in matches if m1.distance < 0.7 * m2.distance]
                        
                        if len(good) > minimum_match and matchesMask.count([1,0])>15:
                            # change to bbox
                            src_pts = np.float32([ keypoints1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
                            dst_pts = np.float32([ keypoints2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
                            # dst_pts[:,0,0] += section[0]
                            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
                            if M is not None:
                                h,w = img1.shape[:2]
                                pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
                                dst = cv2.perspectiveTransform(pts,M)
                                # check size of articles on row
                                pix_width  = (dst[3,0,0] - dst[1,0,0])
                                pix_height = (dst[1,0,1] - dst[3,0,1])
                                resp1 = self.nameRequest(str(article))
                                real_width =  resp1.width
                                real_height = resp1.height
                                # check bbox size
                                if (pix_height <= ((real_height * self.pixelRation) + self.pixelRange ) and pix_height >= ((real_height * self.pixelRation) - self.pixelRange )) and (pix_width <= ((real_width * self.pixelRation) + self.pixelRange ) and pix_width >= ((real_width * self.pixelRation) - self.pixelRange )):
                                    print(str(article) + ' number of matches: ' + str(matchesMask.count([1,0])) + ' / ' + str(len(good)))
                                    new_item = item()
                                    new_item.confidence = len(good)
                                    new_item.name = str(article)
                                    new_item.box_2d.x1 = dst[1,0,0]
                                    new_item.box_2d.y1 = dst[3,0,1]
                                    new_item.box_2d.x2 = dst[3,0,0]
                                    new_item.box_2d.y2 = dst[1,0,1]
                                    new_item.header = self.header
                                    self.item_pub.publish(new_item)
                                    # img3 = cv2.rectangle(stream, (int(new_item.box_2d.x1),int(new_item.box_2d.y1)),(int(new_item.box_2d.x2),int(new_item.box_2d.y2)), (0,255,0),2)
                                    # cv2.imshow("results",img3)
                                    # cv2.waitKey(10)
                                else:
                                    print("bbox out of range -- height: " + str(real_height*self.pixelRation) + " -- width: " + str(real_width*self.pixelRation))
                                    print(str(pix_height) + " -- " + str(pix_width))
                iteration += 1   
            self.finished_pub.publish(True)


    def shutdown(self):
        plt.close()

    def classification(self):
        rospy.init_node('detection_node')
        # rospy.Subscriber('find_item', String, self.item_callback)
        # d435 and kinova
        rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback_matching)
        
        rospy.Subscriber('/classification_search', String, self.processing_callback)

        self.item_pub = rospy.Publisher('/new_item', item, queue_size=5)
        self.finished_pub = rospy.Publisher('/classification_finished', Bool, queue_size=5)

        rospy.spin() 
        rospy.on_shutdown(self.shutdown)


if __name__ == '__main__':
    node = find_item()
    node.classification()



# detector = cv2.ORB_create()

# # find the keypoints and descriptors with ORB
# kp1, des1 = detector.detectAndCompute(img1,None)
# kp2, des2 = detector.detectAndCompute(img2,None)

# # create BFMatcher object
# bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
# # Match descriptors.
# matches = bf.match(des1,des2)
# # Sort them in the order of their distance.
# matches = sorted(matches, key = lambda x:x.distance)

# good_matches = matches[:20]

# src_pts = np.float32([ kp1[m.queryIdx].pt for m in good_matches     ]).reshape(-1,1,2)
# dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good_matches ]).reshape(-1,1,2)
# M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,3.0)
# matchesMask = mask.ravel().tolist()
# h,w = img1.shape[:2]
# pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)

# dst = cv2.perspectiveTransform(pts,M)
# dst += (w, 0)  # adding offset

# draw_params = dict(matchColor = (0,255,0), # draw matches in green color
#             singlePointColor = None,
#             matchesMask = matchesMask, # draw only inliers
#             flags = 2)

# img3 = cv2.drawMatches(img1,kp1,img2,kp2,good_matches, None,**draw_params)
# img3 = cv2.polylines(img3, [np.int32(dst)], True, (0,0,255),3, cv2.LINE_AA)


# BFMatcher with default params
# bf = cv2.BFMatcher()
# matches = bf.knnMatch(des1,des2,k=2)
# # Apply ratio test
# good_matches = []
# for m,n in matches:
#     if m.distance < 0.75*n.distance:
#         good_matches.append([m])

# img3 = cv2.drawMatchesKnn(img1,kp1,img2,kp2,good_matches,None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)