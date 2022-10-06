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


class find_item:
    def __init__(self):
        self.article_path = ''
        self.bbox_array = []
        cwd = rospkg.RosPack().get_path('paul_vision')
        self.articles_folder = cwd + '/cannes/'
        self.articles_array = []
        self.iterations = 0
        self.camera_stream = None

    def item_callback(self, name):
        #find unique article
        cwd = rospkg.RosPack().get_path('paul_vision')
        self.article_path = cwd + '/articles/' + name.data + '.png'
        print(self.article_path)

        # ajouter service avec database pour avoir autres informations
        # height, width
        # ajouter images sur le même étage seulement
        self.articles_array = []
        for images in os.listdir(self.articles_folder):
            if (images.endswith(".png")):
                # image_path = self.articles_folder + images
                self.articles_array.append(images)

    def image_callback_matching(self, image):
        # to use without the segmentation pre-process
        try: 
            self.camera_stream = bridge.imgmsg_to_cv2(image, 'bgr8')
            self.header = image.header
        except CvBridgeError as e:
            print("CvBridge could not convert images from realsense to opencv")
        
    
    def processing_callback(self, msg):
        # print('here')
        if msg.data:
            # decision arbitraire à revoir
            sections = [[0,200], [110,310], [220,420], [330, 530], [440,640]]
            # decision arbitraire à revoir
            minimum_match = 40
            article_match = ''
            if self.camera_stream is not None:
                for section in sections:
                    stream = self.camera_stream.copy()
                    img2 = stream[0:,section[0]:section[1]].copy()
                    img2 = cv2.resize(img2,(img2.shape[1],img2.shape[0]))
                    img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

                    sift = cv2.xfeatures2d.SIFT_create()
                    keypoints2, descriptors2 = sift.detectAndCompute(img2, None)
                    # print('************************')
                    # iteration de tous les articles (ajouter étage par étage)
                    for article in self.articles_array:
                        # print(article)
                        article_path = self.articles_folder + article
                        img1 = cv2.imread(article_path,cv2.IMREAD_GRAYSCALE) # queryImage
                        img1 = cv2.normalize(img1, None, 0, 255, cv2.NORM_MINMAX)
                        
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
                        
                        # if matchesMask.count([1,0]) > minimum_match:
                        if len(good) > minimum_match and matchesMask.count([1,0])>15:
                            # canvas = stream.copy()
                            article_match = str(article)
                            
                            src_pts = np.float32([ keypoints1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
                            dst_pts = np.float32([ keypoints2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
                            dst_pts[:,0,0] += section[0]
                            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
                            if M is not None:
                                h,w = img1.shape[:2]
                                pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
                                dst = cv2.perspectiveTransform(pts,M)
                                # img3 = cv2.polylines(canvas,[np.int32(dst)],True,(0,255,0),3, cv2.LINE_AA)
                                # img3 = cv2.rectangle(canvas, (int(dst[0,0,0]),int(dst[0,0,1])),(int(dst[2,0,0]),dst[2,0,1]), (0,0,255),2)
                                # print(dst[:,0])
                                # draw_params = dict (matchColor = (0,0,255), singlePointColor = (0,255,0), matchesMask = matchesMask, flags=0 )
                                # flann_matches =cv2.drawMatchesKnn(img1, keypoints1, img3, keypoints2, matches, None,**draw_params)
                                
                                # print(str(article_match) + ' number of matches: ' + str(matchesMask.count([1,0])) + ' / ' + str(len(good)))
                                new_item = item()
                                new_item.confidence = len(good)
                                new_item.name = str(article)
                                new_item.box_2d.x1 = dst[1,0,0]
                                new_item.box_2d.y1 = dst[3,0,1]
                                new_item.box_2d.x2 = dst[3,0,0]
                                new_item.box_2d.y2 = dst[1,0,1]
                                new_item.header = self.header
                                self.item_pub.publish(new_item)
                                img3 = cv2.rectangle(stream, (int(new_item.box_2d.x1),int(new_item.box_2d.y1)),(int(new_item.box_2d.x2),int(new_item.box_2d.y2)), (0,255,0),2)
                                cv2.imshow("results",img3)
                                cv2.waitKey(10)
                # reset somewhere??
                self.iterations += 1
                if self.iterations > 2:
                    self.finished_pub.publish(True)
                else:
                    self.finished_pub.publish(False)


    def shutdown(self):
        plt.close()

    def classification(self):
        rospy.init_node('detection_node')
        rospy.Subscriber('find_item', String, self.item_callback)
        rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback_matching)
        rospy.Subscriber('/classification_search', Bool, self.processing_callback)

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