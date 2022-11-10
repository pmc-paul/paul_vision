#!/usr/bin/env python2.7

import rospy
import smach
import smach_ros
import numpy as np
from database.srv import *
from std_msgs.msg import Bool
from paul_vision.msg import item

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
        smach.State.__init__(self, outcomes=['moveArm','moveElevation','objectNotFound','FeatureMatching'], input_keys=['articleId'])
        self.elevation_count = 0
        self.etagere_state = np.zeros((3,3))
        self.currentLevel = 1 # check level in execute with elevation service

    def execute(self, userdata):
        rospy.wait_for_service('sqlRequest')
        try:
            db_request = rospy.ServiceProxy('sqlRequest', itemDetails)
            resp1 = db_request(userdata.articleId)
            self.articleLevel =  resp1.level
            print(self.articleLevel)
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
        #     return 'objectNotFound'
        return 'FeatureMatching'

class moveArm(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['logic_dispatch'], input_keys=['article'])
        self.counter = 0

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
        smach.State.__init__(self, outcomes=['notFound','grab'], output_keys=['item_info'], input_keys=['article'])        
        self.request_pub = rospy.Publisher('/classification_search', Bool, queue_size=1)
        self.identified_sub = rospy.Subscriber('/new_item', item, self.new_items_callback)
        self.finished_sub = rospy.Subscriber('/classification_finished', Bool, self.finished_callback)
        self.finished = False
        self.items_found = []

    def execute(self, userdata):
        rospy.loginfo('Executing state matching')

        while(not self.finished):
            sleep(0.5)
        
        if len(self.items_found)>0:
            for item in self.items_found:
                if item.item_id == userdata.articleId:
                    userdata.item_info = item
                    return 'grab'
        else:
            return 'notFound'

    def new_items_callback(self, msg):
        self.items_found.append(msg.data)

    def finished_callback(self, msg):
        self.finished = msg.data

class notFound(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['logic_dispatch'])

    def execute(self, userdata):
        rospy.loginfo('Object not found here')
        rospy.sleep(2)
        return 'logic_dispatch'
        
class Grab(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['objectFound'], input_keys=['item_info'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Grab')
        rospy.sleep(5)
        return 'objectFound'
# main
def main():
    rospy.init_node('smach_vision')
    article = rospy.get_param('~article')
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=[ 'ObjectFound', 'ObjectNotFound'])
    sm.userdata.articleId = article
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
                                transitions={'objectFound':'objectFound'})
        smach.StateMachine.add('notFound', notFound(),  
                                transitions={'logic_dispatch':'logic_dispatch'})
        smach.StateMachine.add('MoveElevation', moveElevation(), 
                               transitions={'logic_dispatch':'logic_dispatch'})
        smach.StateMachine.add('logic_dispatch', logic_dispatch(),
                                transitions={'moveArm':'MoveArm','moveElevation':'MoveElevation','objectNotFound':'objectNotFound','FeatureMatching':'FeatureMatching'})   

    # introspection server for smach GUI
    sis = smach_ros.IntrospectionServer('server_vision', sm, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()