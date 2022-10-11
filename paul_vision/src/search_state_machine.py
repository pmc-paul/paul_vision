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
        smach.State.__init__(self, outcomes=['moveArm','moveElevation','End','FeatureMatching'], input_keys=['articleId'])
        self.counter = 0
        self.elevation_count = 0
        self.etagere_state = np.zeros((3,3))
        self.currentLevel = 0
        self.articleLevel = database_client(userdata.articleId)

    def database_client(articleId):
        rospy.wait_for_service('sqlRequest')
        try:
            db_request = rospy.ServiceProxy('sqlRequest', itemDetails)
            resp1 = db_request(articleId)
            return resp1.level
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def execute(self, userdata):
        rospy.sleep(1)
        # check level with elevateur
        self.counter += 1
        if self.counter < 3:
            rospy.sleep(2)
            return 'moveArm'
        elif self.elevation_count<1:
            self.elevation_count +=1
            return 'moveElevation'
        else:
            return 'FeatureMatching'

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
        smach.State.__init__(self, outcomes=['End'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Grab')
        rospy.sleep(5)
        return 'End'

class End(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['objectFound','objectNotFound'])
    
    def execute(self, userdata):
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
                                transitions={'End':'End'})
        smach.StateMachine.add('notFound', notFound(),  
                                transitions={'logic_dispatch':'logic_dispatch'})
        smach.StateMachine.add('MoveElevation', moveElevation(), 
                               transitions={'logic_dispatch':'logic_dispatch'})
        smach.StateMachine.add('End', End(), 
                               transitions={'objectFound':'ObjectFound','objectNotFound':'ObjectNotFound'})
        smach.StateMachine.add('logic_dispatch', logic_dispatch(),
                                transitions={'moveArm':'MoveArm','moveElevation':'MoveElevation','End':'End','FeatureMatching':'FeatureMatching'})   

    # introspection server for smach GUI
    sis = smach_ros.IntrospectionServer('server_vision', sm, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()