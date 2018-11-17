#!/usr/bin/env python
import rospy
from Controller import Controller

def initNode():

    # launch a controller node @30Hz
    rospy.init_node('controller_node', anonymous=True)
    r = rospy.Rate(30)
    controller = Controller()
    while not rospy.is_shutdown():
        # access controller methods
        controller.set_acc(0, 0)
        r.sleep()

if __name__ == '__main__':
    try:
        initNode()
    except rospy.ROSInterruptException:
        pass