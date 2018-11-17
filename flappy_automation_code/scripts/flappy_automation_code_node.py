#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
from Controller import Flappy, Controller

# Publisher for sending acceleration commands to flappy bird
pub_acc_cmd = rospy.Publisher('/flappy_acc', Vector3, queue_size=1)

def initNode():
    # Here we initialize our node running the automation code
    rospy.init_node('flappy_automation_code', anonymous=True)

    # Subscribe to topics for velocity and laser scan from Flappy Bird game
    flappy = Flappy()

    # Ros spin to prevent program from exiting
    rospy.spin()

if __name__ == '__main__':
    try:
        initNode()
    except rospy.ROSInterruptException:
        pass