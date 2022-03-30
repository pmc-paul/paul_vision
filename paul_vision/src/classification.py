#!/usr/bin/env python3
import numpy as np
import cv2
import matplotlib.pyplot as plt
import rospy
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from paul_vision.msg import BBox2d, BBox2d_array
from sensor_msgs.msg import Image
import os
bridge = CvBridge()
import rospkg

# dictionnary of objets
# bbox coordinates
# name of object with most match
# number of matches

class find_item:
    def __init__(self):
        self.article_path = ''
        self.array = []

    def item_callback(self, name):
        #find article path
        cwd = rospkg.RosPack().get_path('paul_vision')
        self.article_path = cwd + '/articles/' + name.data + '.png'
        print(self.article_path)

    def image_callback_matching(self, image):
        if self.article_path != '':
            try: 
                stream = bridge.imgmsg_to_cv2(image, 'bgr8')
            except CvBridgeError as e:
                print("CvBridge could not convert images from realsense to opencv")
            # print(self.article_path)
            img1 = cv2.imread(self.article_path,cv2.IMREAD_GRAYSCALE) # queryImage
            img1 = cv2.normalize(img1, None, 0, 255, cv2.NORM_MINMAX)
            img2 = cv2.cvtColor(stream, cv2.COLOR_BGR2GRAY)
            
            sift = cv2.xfeatures2d.SIFT_create()

            keypoints1, descriptors1 = sift.detectAndCompute(img1, None)
            keypoints2, descriptors2 = sift.detectAndCompute(img2, None)

            FLAN_INDEX_KDTREE = 0
            index_params = dict (algorithm = FLAN_INDEX_KDTREE, trees=5)
            search_params = dict (checks=50)

            flann = cv2.FlannBasedMatcher(index_params, search_params)
            matches = flann.knnMatch (descriptors1, descriptors2, k=2)

            matchesMask = [[0,0] for i in range(len(matches))]
            for i,(m1, m2) in enumerate (matches):
                if m1.distance < 0.5 * m2.distance:
                    matchesMask[i] = [1,0]

            draw_params = dict (matchColor = (0,0,255), singlePointColor = (0,255,0), matchesMask = matchesMask, flags=0 )
            print(matchesMask.count([1,0]))   
            flann_matches =cv2.drawMatchesKnn(img1, keypoints1, img2, keypoints2, matches, None,**draw_params)
            cv2.imshow('result',flann_matches)

            # img3 = cv2.polylines(img3, [np.int32(dst)], True, (0,0,255),3, cv2.LINE_AA)
            # plt.imshow(img3),plt.show()
            # cv2.imshow("result", img3)
            cv2.waitKey(30)


    def image_callback_bbox(self, image):
        try: 
            stream = bridge.imgmsg_to_cv2(image, 'bgr8')
        except CvBridgeError as e:
            print("CvBridge could not convert images from realsense to opencv")
        if len(self.array) and self.article_path != '':
            for box in self.array:
                img1 = cv2.imread(self.article_path,cv2.IMREAD_GRAYSCALE) # queryImage
                img1 = cv2.normalize(img1, None, 0, 255, cv2.NORM_MINMAX)
                
                img2 = stream[int(box.y1):int(box.y2),int(box.x1):int(box.x2)].copy()
                img2 = cv2.resize(img2,(img2.shape[1],img2.shape[0]))
                img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

                sift = cv2.xfeatures2d.SIFT_create()

                keypoints1, descriptors1 = sift.detectAndCompute(img1, None)
                keypoints2, descriptors2 = sift.detectAndCompute(img2, None)

                FLAN_INDEX_KDTREE = 0
                index_params = dict (algorithm = FLAN_INDEX_KDTREE, trees=5)
                search_params = dict (checks=50)

                flann = cv2.FlannBasedMatcher(index_params, search_params)
                matches = flann.knnMatch (descriptors1, descriptors2, k=2)

                matchesMask = [[0,0] for i in range(len(matches))]
                for i,(m1, m2) in enumerate (matches):
                    if m1.distance < 0.5 * m2.distance:
                        matchesMask[i] = [1,0]

                draw_params = dict (matchColor = (0,0,255), singlePointColor = (0,255,0), matchesMask = matchesMask, flags=0 )
                print(matchesMask.count([1,0]))   
                flann_matches =cv2.drawMatchesKnn(img1, keypoints1, img2, keypoints2, matches, None,**draw_params)
                cv2.imshow('result',flann_matches)

                # img3 = cv2.polylines(img3, [np.int32(dst)], True, (0,0,255),3, cv2.LINE_AA)
                # plt.imshow(img3),plt.show()
                # cv2.imshow("result", img3)
                cv2.waitKey(30)

    def bbox_callback(self, bbox):
        self.array = bbox.boxes
        

    def shutdown(self):
        plt.close()

    def listener(self):
        rospy.init_node('detection_node')
        # global self.article_path
        rospy.Subscriber('find_item', String, self.item_callback)
        rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback_matching)
        rospy.Subscriber('classification_bounding_boxes', BBox2d_array, self.bbox_callback)

        rospy.spin() 
        rospy.on_shutdown(self.shutdown)

# def main():

#     rospy.spin()


if __name__ == '__main__':
    node = find_item()
    node.listener()



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