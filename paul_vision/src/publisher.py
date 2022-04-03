#!/usr/bin/env python3
import rospy


#!/usr/bin/env python
# license removed for brevity
import rospy
from paul_vision.msg import BBox2d, BBox2d_array

def talker():
    boxes_pub = rospy.Publisher('classification_bounding_boxes', BBox2d_array, queue_size = 1)
    rospy.init_node('publishing_node')
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        box = BBox2d()
        box_msg = BBox2d_array()

        box.x1= 100.0
        box.y1= 100.0
        box.x2= 550.0
        box.y2= 400.0
        box_msg.boxes.append(box)

        # box2 = BBox2d()
        # box2.x1= 150.0
        # box2.y1= 200.0
        # box2.x2= 300.0
        # box2.y2= 400.0
        # box_msg.boxes.append(box2)

        # box3 = BBox2d()
        # box3.x1= 0.0
        # box3.y1= 200.0
        # box3.x2= 150.0
        # box3.y2= 400.0
        # box_msg.boxes.append(box3)

        # box4 = BBox2d()
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