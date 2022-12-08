#!/usr/bin/env python3

import roslaunch
import rosnode
import rospy
import rospkg
from std_msgs.msg import String


global currentId
currentId = ''

def request_callback(article):
    # if sm not running
    print('new request id: '+article.data)
    global currentId 
    currentId= article.data


def main():
    rospy.init_node('vision_sm_init')
    #subscriber to web app
    rospy.Subscriber('/ui_article_request', String, request_callback)
    while(not rospy.is_shutdown()):
        node_list = rosnode.get_node_names()
        if ('/search_state_machine' not in node_list and currentId is not ''):
            print("/smach_vision not running")
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            # change path
            path = [rospkg.RosPack().get_path('paul_vision') + '/launch/vision_sm.launch']
            arg = ['article:=' + str(currentId)]

            roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(path), arg)]
            parent = roslaunch.parent.ROSLaunchParent(uuid, path)
            parent.start()
            rospy.sleep(3)
        rospy.sleep(1)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass