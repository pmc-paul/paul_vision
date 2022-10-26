#!/usr/bin/env python2.7

import rospy
import smach
import smach_ros
import numpy as np
from database.srv import *

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
        self.currentLevel = 0 # check level in execute with elevation service
        rospy.wait_for_service('sqlRequest')
        try:
            db_request = rospy.ServiceProxy('sqlRequest', itemDetails)
            resp1 = db_request(articleId)
            self.articleLevel =  resp1.level
            print(self.articleLevel)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def execute(self, userdata):
        # rospy.sleep(1)
        goodLevel = False
        # move elevateur first
        if(np.any(self.etagere_state[self.articleLevel] == 0)):
            # check level with elevateur
            if self.currentLevel == self.articleLevel:
                goodLevel = True
            else:
                goodLevel = False
                return 'moveElevation'
                #move elevateur service
                # wait   
        #else to search other level
        else:
            for level in range(len(self.etagere_state[:0])):
                if np.any(self.etagere_state[level] == 0) :
                    # add user data
                   return 'moveElevation' 
        # move arm if necessary
        counter = 0
        pos_to_search = -1
        for position in self.etagere_state[1]:
            if position == 0:
                pos_to_search = counter
            counter += 1

        # check arm position and move arm

        if pos_to_search != -1:
            return 'FeatureMatching'
        else:
            return 'objectNotFound'

class moveArm(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['logic_dispatch'])
        self.counter = 0

    def execute(self, userdata):
        return 'logic_dispatch'

class moveElevation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['logic_dispatch'])

    def execute(self, userdata):
        return 'logic_dispatch'

# define state Bar
class matching(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['notFound','grab'])

    def execute(self, userdata):
        rospy.loginfo('Executing state matching')
        rospy.sleep(5)
        return 'notFound'

class notFound(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['logic_dispatch'])

    def execute(self, userdata):
        rospy.loginfo('Object not found here')
        rospy.sleep(2)
        return 'logic_dispatch'
        
class Grab(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['objectFound'])

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