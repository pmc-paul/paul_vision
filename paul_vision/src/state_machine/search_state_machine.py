#!/usr/bin/env python2.7

import rospy
import smach
import smach_ros
import numpy as np
from math import pi
from database.srv import *
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Pose
from paul_vision.msg import item, classified_items
from paul_vision.srv import check
from paul_manipulation.srv import ArmPosition, ElevationVision, ArmPositionJoint

SHELF_1_POS_0 = (1.2569861273, 5.3750904974, 1.3330824827, 4.5211008944, 1.3259266327, 0.70389128733)
SHELF_1_POS_1 = (1.2569861273, 5.3750904974, 1.3330824827, 4.5211008944, 1.3259266327, 0.70389128733)
SHELF_2_POS_0 = (1.2569861273, 5.3750904974, 1.3330824827, 4.5211008944, 1.3259266327, 0.70389128733)
SHELF_2_POS_1 = (1.2569861273, 5.3750904974, 1.3330824827, 4.5211008944, 1.3259266327, 0.70389128733)
SHELF_3_POS_0 = (0.81908301796, 5.8243382468, 2.4972170938, 3.978303497, 1.4402456987, 1.4486232792)
SHELF_3_POS_1 = (1.2569861273, 5.3750904974, 1.3330824827, 4.5211008944, 1.3259266327, 0.70389128733)
SHELF_4_POS_0 = (0.96638880683, 0.23038346126, 2.0844467257, 4.5211008944, 0.98942715296, 0.32306044454)
SHELF_4_POS_1 = (1.3693853311, 5.6710983385, 1.029046127, 4.7001716756, 1.369734397, 0.053756140961)

class start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['logic_dispatch'])

    def execute(self, userdata):
        return 'logic_dispatch'

class logic_dispatch(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['moveArm','moveElevation','ObjectNotFound','FeatureMatching'], input_keys=['articleId', 'searchPos'], output_keys=['articleName','searchPos','level'])
        self.elevation_count = 0
        self.etagere_state = np.zeros((5,2))
        self.currentLevel = 0 # check level in execute with elevation service

    def execute(self, userdata):
        rospy.wait_for_service('sqlRequest')
        try:
            db_request = rospy.ServiceProxy('sqlRequest', itemDetails)
            resp1 = db_request(str(userdata.articleId))
            self.articleLevel =  resp1.level
            userdata.articleName = resp1.image_name
            userdata.level = self.articleLevel # (int)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        # move elevateur first
        if self.currentLevel != int(self.articleLevel):
            self.currentLevel = int(self.articleLevel)
            # return 'moveElevation'

        # move arm if necessary
        # counter = 0
        # for position in self.etagere_state[int(self.articleLevel)]:
        #     if position == 0:
        #         position = 1
        #         userdata.searchPos = counter
                # break
            # counter += 1
        # if counter!=0:
            # return 'moveArm'
        if userdata.searchPos < 2:
            return 'moveArm'
        else:
            return 'ObjectNotFound'

        # # check arm position and move arm

        # if pos_to_search != -1:
        #     return 'FeatureMatching'
        # else:
        #     return 'ObjectNotFound'

class moveArm(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['logic_dispatch','FeatureMatching'], input_keys=['searchPos', 'level'])
        self.counter = 0
        self.posemsg = [0,0,0,0,0,0]
        self.pose_found = False
        # self.arm_pub = rospy.Publisher('/arm_position_request', Pose, queue_size=1)

    def execute(self, userdata):
        # rospy.sleep(1)
        # service to arm with userdata.searchPos (int to Pose)
        self.findPos(userdata)
        response = False
        while(not self.pose_found):
            rospy.sleep(0.1)
        rospy.wait_for_service('/my_gen3/arm_position_grab')
        try:
            move_request = rospy.ServiceProxy('/my_gen3/arm_position', ArmPositionJoint)
            response = move_request(self.posemsg).success
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        while(not response):
            rospy.sleep(0.05)
        return 'FeatureMatching'

    def findPos(self, userdata):
        # robot (acier) a 44 cm de l'etagere
        # only 3 and 4
        # a retravailler lol
        if userdata.level == 1:
            if userdata.searchPos == 0:
                self.pose_found = True
            elif userdata.searchPos == 1:
                self.pose_found = True
        elif userdata.level == 2:
            if userdata.searchPos == 0:
                self.pose_found = True
            elif userdata.searchPos == 1:
                self.pose_found = True
        elif userdata.level == 3:
            if userdata.searchPos == 0:
                self.posemsg = [-(2*pi-6.015277267), -(2*pi-5.679301386), 2.181661565, 2.8591983806, 1.2388347031, 1.6786576746]
                self.pose_found = True
            elif userdata.searchPos == 1:
                self.posemsg = [0.7189011189, -(2*pi-5.518207496), 1.899267292, -(2*pi-3.9196604341), 1.2187634167, 1.2547171993]
                self.pose_found = True
        elif userdata.level == 4:
            if userdata.searchPos == 0:
                self.posemsg = [-(2*pi-5.5955255819), 0.03490658504, 1.8421950255, 1.8606955156, 0.72623150175, 2.7501153024]
                self.pose_found = True
            elif userdata.searchPos == 1:
                self.posemsg = [-(2*pi-5.5955255819), 0.03490658504, 1.8421950255, 1.8606955156, 0.72623150175, 2.7501153024]
                self.pose_found = True


class moveElevation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['logic_dispatch'], input_keys=['level'])

    def execute(self, userdata):
        # rospy.sleep(1)
        #service to elevation with userdata.level (int)
        rospy.wait_for_service('/my_gen3/vision_elevation_first')
        try:
            elevation_request = rospy.ServiceProxy('/my_gen3/vision_elevation_first', ElevationVision)
            msg = ElevationVision()
            
            response = elevation_request(msg).response
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        return 'logic_dispatch'

# define state Bar
class matching(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['notFound','grab'], output_keys=['item_info'], input_keys=['articleName', 'articleId','item_info'])        
        self.request_pub = rospy.Publisher('/classification_search', String, queue_size=1)
        rospy.Subscriber('/classified_items', classified_items, self.classified_articles_callback)

    def execute(self, userdata):
        rospy.loginfo('searching for: ' + str(userdata.articleName))
        self.finished = False
        self.items_found = []
        rospy.wait_for_service('check_classification')
        response = False
        counter =0
        while(not response and counter<5):
            try:
                classification_request = rospy.ServiceProxy('check_classification', check)
                response = classification_request(True).response
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
                counter +=1
                rospy.sleep(0.5)

        self.request_pub.publish(userdata.articleName)

        while(not self.finished):
            rospy.sleep(0.05)
            print("Classifying ...")
        
        found = False
        if len(self.items_found)>0:
            for item in self.items_found:
                print(item.item_id)
                if item.item_id == str(userdata.articleId):
                    userdata.item_info = item
                    print(userdata.item_info)
                    found = True
        if found:
            return 'grab'
        else:
            return 'notFound'

    def classified_articles_callback(self, msg):
        self.finished = True
        self.items_found = msg.items

class notFound(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['logic_dispatch'], input_keys=['searchPos'], output_keys=['searchPos'])

    def execute(self, userdata):
        rospy.loginfo('Object not found here')
        userdata.searchPos += 1
        # rospy.sleep(2)
        return 'logic_dispatch'
        
class Grab(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ObjectFound'], input_keys=['item_info'])
        # self.arm_pub = rospy.Publisher('/arm_position_request', Pose, queue_size=1)
        # envoyer topic a l'app web

    def execute(self, userdata):
        rospy.loginfo('Executing state Grab')
        item = userdata.item_info
        pose3d = item.box_3d
        pose_z_arm = pose3d.centery - 0.02 + (item.height_offset * 0.01)
        ### change x,y selon pos actuelle du robot
        pose_y_arm = pose3d.centerx - 0.01 # add when depth negative
        pose_x_arm = pose3d.depth + 0.11 # -0.11 when depth negative
        # live trop a droite pour item a droite
        # gauche pas assez a gauche
        rospy.wait_for_service('/my_gen3/arm_position_grab')
        try:
            grab_request = rospy.ServiceProxy('/my_gen3/arm_position_grab', ArmPosition)
            posemsg = Pose()
            posemsg.position.x = pose_x_arm
            posemsg.position.y = pose_y_arm
            posemsg.position.z = pose_z_arm
            grip = (10-item.width)/10
            if grip<0:
                grip = 1
            response = grab_request(posemsg, grip).success
            print(response)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        # rospy.sleep(5)
        return 'ObjectFound'
# main
def main():
    rospy.init_node('smach_vision')
    articleId = rospy.get_param('~article')
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=[ 'ObjectFound', 'ObjectNotFound'])
    sm.userdata.articleId = articleId
    sm.userdata.searchPos = 0
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('start', start(), 
                               transitions={'logic_dispatch':'logic_dispatch'})
        smach.StateMachine.add('MoveArm', moveArm(), 
                               transitions={'logic_dispatch':'logic_dispatch', 'FeatureMatching':'FeatureMatching'})
        smach.StateMachine.add('FeatureMatching', matching(), 
                               transitions={'notFound':'notFound', 'grab':'Grab'})
        smach.StateMachine.add('Grab', Grab(),  
                                transitions={'ObjectFound':'ObjectFound'})
        smach.StateMachine.add('notFound', notFound(),  
                                transitions={'logic_dispatch':'logic_dispatch'})
        smach.StateMachine.add('MoveElevation', moveElevation(), 
                               transitions={'logic_dispatch':'logic_dispatch'})
        smach.StateMachine.add('logic_dispatch', logic_dispatch(),
                                transitions={'moveArm':'MoveArm','moveElevation':'MoveElevation','ObjectNotFound':'ObjectNotFound','FeatureMatching':'FeatureMatching'})   

    # introspection server for smach GUI
    sis = smach_ros.IntrospectionServer('server_vision', sm, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()