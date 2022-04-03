#!/usr/bin/env python3
import rospy


#!/usr/bin/env python
# license removed for brevity
import rospy
from paul_vision.msg import BBox3d, BBox3d_array

def talker():
    boxes_pub = rospy.Publisher('/bounding_boxes_3d', BBox3d_array, queue_size = 1)
    rospy.init_node('publishing_node')
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        box = BBox3d()
        box_msg = BBox3d_array()

        box.x1= -0.22775453186
        box.y1= 0.0768706665039
        box.x2= -0.123533912659
        box.y2= 0.145851394653
        box.depth= 0.914
        box_msg.boxes.append(box)


        # box2 = BBox3d()
        # box2.x1= 0.022041727066
        # box2.y1= 0.308638977051
        # box2.x2= -0.0491593360901
        # box2.y2= 0.145552215576
        # box2.depth= 0.943
        # box_msg.boxes.append(box2)

        # box3 = BBox3d()
        # box3.x1= -0.527137084961
        # box3.y1= 0.341374725342
        # box3.x2= -0.477479309082
        # box3.y2= 0.169851135254
        # box3.depth = 1.019
        # box_msg.boxes.append(box3)

        # box4 = BBox3d()
        # box4.x1= 400.0
        # box4.y1= 200.0
        # box4.x2= 550.0
        # box4.y2= 400.0
        # box_msg.boxes.append(box4)
    
        boxes_pub.publish(box_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

  
#     x1: 0.410065795898
#     y1: -0.325290283203
#     x2: 0.407632720947
#     y2: -0.163530441284
#     depth: 1.013
#   - 
#     x1: 0.0998450622559
#     y1: -0.305055480957
#     x2: 0.17083821106
#     y2: -0.141982589722
#     depth: 0.911
#   - 
#     x1: 0.694463256836
#     y1: -0.365354705811
#     x2: 0.602248657227
#     y2: -0.166193435669
#     depth: 0.921
#   - 
#     x1: 0.236210891724
#     y1: -0.32592300415
#     x2: 0.0
#     y2: -0.0
#     depth: 0.915
