#!/usr/bin/env python2.7

import rospy
import smach
import smach_ros
import numpy as np
from database.srv import *
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Pose
from paul_vision.msg import item, classified_items
from paul_vision.srv import check
from paul_manipulation.srv import ArmPosition

class start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['logic_dispatch'])

    def execute(self, userdata):
        return 'logic_dispatch'

class logic_dispatch(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['moveArm','moveElevation','ObjectNotFound','FeatureMatching'], input_keys=['articleId'], output_keys=['articleName','searchPos','level'])
        self.elevation_count = 0
        self.etagere_state = np.zeros((3,3))
        self.currentLevel = 3 # check level in execute with elevation service

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
        # # rospy.sleep(1)
        # move elevateur first
        if self.currentLevel != int(self.articleLevel):
            return 'moveElevation'

        # move arm if necessary
        # counter = 0
        # for position in self.etagere_state[int(self.articleLevel)]:
        #     if position == 0:
        #         userdata.searchPos = counter
        #         return 'moveArm'
        #     counter += 1

        # # check arm position and move arm

        # if pos_to_search != -1:
        #     return 'FeatureMatching'
        # else:
        #     return 'ObjectNotFound'
        return 'FeatureMatching'

class moveArm(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['logic_dispatch'], input_keys=['searchPos', 'level'])
        self.counter = 0
        self.arm_pub = rospy.Publisher('/arm_position_request', Pose, queue_size=1)

    def execute(self, userdata):
        rospy.sleep(1)
        # service to arm with userdata.searchPos (int to Pose)
        return 'logic_dispatch'

    def findPos(self, userdata):
        if userdata.level == 0:
            if userdata.searchPos == 0:
                rospy.sleep(0.5)
            elif userdata.searchPos == 1:
                rospy.sleep(0.5)
            elif userdata.searchPos == 2:
                rospy.sleep(0.5)
        elif userdata.level == 1:
            if userdata.searchPos == 0:
                rospy.sleep(0.5)
            elif userdata.searchPos == 1:
                rospy.sleep(0.5)
            elif userdata.searchPos == 2:
                rospy.sleep(0.5)
        elif userdata.level == 2:
            if userdata.searchPos == 0:
                rospy.sleep(0.5)
            elif userdata.searchPos == 1:
                rospy.sleep(0.5)
            elif userdata.searchPos == 2:
                rospy.sleep(0.5)


class moveElevation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['logic_dispatch'], input_keys=['level'])

    def execute(self, userdata):
        rospy.sleep(1)
        #service to elevation with userdata.level (int)
        return 'logic_dispatch'

# define state Bar
class matching(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['notFound','grab'], output_keys=['item_info'], input_keys=['articleName', 'articleId','item_info'])        
        self.request_pub = rospy.Publisher('/classification_search', String, queue_size=1)
        rospy.Subscriber('/classified_items', classified_items, self.classified_articles_callback)
        self.finished = False
        self.items_found = []

    def execute(self, userdata):
        rospy.loginfo('searching for: ' + str(userdata.articleName))
        
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
            rospy.sleep(0.2)
        
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
        smach.State.__init__(self, outcomes=['logic_dispatch'])

    def execute(self, userdata):
        rospy.loginfo('Object not found here')
        rospy.sleep(2)
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
        pose_z_arm = pose3d.centery - 0.02
        ### change x,y selon pos actuelle du robot
        pose_y_arm = pose3d.centerx - 0.009970000013709068
        pose_x_arm = pose3d.depth - 0.12

        rospy.wait_for_service('/my_gen3/arm_position')
        try:
            grab_request = rospy.ServiceProxy('/my_gen3/arm_position', ArmPosition)
            posemsg = ArmPosition()
            posemsg.pose.position.x = pose_x_arm
            posemsg.pose.position.y = pose_y_arm
            posemsg.pose.position.z = pose_z_arm
            posemsg.grip = (8-item.width)/8
            response = grab_request(posemsg).success
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
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('start', start(), 
                               transitions={'logic_dispatch':'logic_dispatch'})
        smach.StateMachine.add('MoveArm', moveArm(), 
                               transitions={'logic_dispatch':'logic_dispatch'})
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