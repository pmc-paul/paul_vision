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

# STATES
# start
# logic
# move to search
## move elevation or move arm
# feature match
# grab item or not found
# end

class start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['logic_dispatch'])

    def execute(self, userdata):
        return 'logic_dispatch'

class logic_dispatch(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['moveArm','moveElevation','ObjectNotFound','FeatureMatching'], input_keys=['articleId'], output_keys=['articleName'])
        self.elevation_count = 0
        self.etagere_state = np.zeros((3,3))
        self.currentLevel = 1 # check level in execute with elevation service

    def execute(self, userdata):
        rospy.wait_for_service('sqlRequest')
        try:
            db_request = rospy.ServiceProxy('sqlRequest', itemDetails)
            resp1 = db_request(str(userdata.articleId))
            self.articleLevel =  resp1.level
            userdata.articleName = resp1.image_name
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        # # rospy.sleep(1)
        # goodLevel = False
        # # move elevateur first
        # if(np.any(self.etagere_state[self.articleLevel] == 0)):
        #     # check level with elevateur
        #     if self.currentLevel == self.articleLevel:
        #         goodLevel = True
        #     else:
        #         goodLevel = False
        #         return 'moveElevation'
        #         #move elevateur service
        #         # wait   
        # #else to search other level
        # else:
        #     for level in range(len(self.etagere_state[:0])):
        #         if np.any(self.etagere_state[level] == 0) :
        #             # add user data
        #            return 'moveElevation' 
        # # move arm if necessary
        # counter = 0
        # pos_to_search = -1
        # for position in self.etagere_state[1]:
        #     if position == 0:
        #         pos_to_search = counter
        #     counter += 1

        # # check arm position and move arm

        # if pos_to_search != -1:
        #     return 'FeatureMatching'
        # else:
        #     return 'ObjectNotFound'
        return 'FeatureMatching'

class moveArm(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['logic_dispatch'], input_keys=['article'])
        self.counter = 0
        self.arm_pub = rospy.Publisher('/arm_position_request', Pose, queue_size=1)

    def execute(self, userdata):
        return 'logic_dispatch'

class moveElevation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['logic_dispatch'], input_keys=['article'])

    def execute(self, userdata):
        print('move elevation')
        return 'logic_dispatch'

# define state Bar
class matching(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['notFound','grab'], output_keys=['item_info'], input_keys=['articleName', 'articleId'])        
        self.request_pub = rospy.Publisher('/classification_search', String, queue_size=1)
        rospy.Subscriber('/classified_items', classified_items, self.classified_articles_callback)
        self.finished = False
        self.items_found = []

    def execute(self, userdata):
        rospy.loginfo('searching for: ' + str(userdata.articleName))
        
        rospy.wait_for_service('check_classification')
        response = False
        while(not response):
            try:
                classification_request = rospy.ServiceProxy('check_classification', check)
                response = classification_request(True).response
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

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
        self.arm_pub = rospy.Publisher('/arm_position_request', Pose, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo('Executing state Grab')
        rospy.sleep(5)
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