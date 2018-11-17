import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3, Point, Pose, Quaternion, PoseStamped
from time import time
from math import cos, sin
import tf

class Flappy():

    def __init__(self):

        self.position = Vector3(0, 0, 0)
        self.timestamp = time()
        self.scan = LaserScan()
        self.velocity = Vector3(0, 0, 0)
        self.pub_pose = rospy.Publisher('/flappy_pose', PoseStamped, queue_size=10)
        rospy.Subscriber("/flappy_vel", Vector3, self.velCallback)
        rospy.Subscriber("/flappy_laser_scan", LaserScan, self.laserScanCallback)

    def velCallback(self, msg):
        # msg has the format of geometry_msgs::Vector3
        # Example of publishing acceleration command on velocity velCallback
        self.velocity = Vector3(msg.x, msg.y, msg.z)
        self.update_position()

    def laserScanCallback(self, msg):
        # msg has the format of sensor_msgs::LaserScan
        # print laser angle and range
        self.scan = msg

    def update_position(self):
        # compute new position from velocity 
        meas_time = time()
        dt = meas_time - self.timestamp
        self.timestamp = meas_time
        self.position.x += self.velocity.x * dt
        self.position.y += self.velocity.y * dt
        br = tf.TransformBroadcaster()
        br.sendTransform((self.position.x, self.position.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "laser_frame",
                     "world")

    def laser_to_pose(self, n):
        # compute the true location of an obstacle
        
        angle = self.scan.angle_min + n * self.scan.angle_increment
        dist = self.scan.ranges[n]
        print "angle:\t" + str(angle) + "\ndist:\t" + str(dist)
        if self.scan.intensities[n]:
            return Vector3( self.position.x + dist * cos(angle),
                            self.position.y + dist * sin(angle),
                            0)
        else:
            return None

    def get_detected_obstacle(self):
        # return detected obstacles absolute position
        # store this in a buffer to build a map

        return [self.laser_to_pose(i) for i in range(9)]

class Controller():

    def __init__(self):

        self.goal = Point(0, 0, 0)
        self.position = PoseStamped()
        self.pub_acc = rospy.Publisher('/flappy_acc', Vector3, queue_size=10)
        rospy.Subscriber("/flappy_pose", PoseStamped, self.poseCallback)

    def poseCallback(self, msg):

        self.position = msg

    def set_acc(self, x, y):
        # publish the acceleration vector (command)
        self.pub_acc.publish(Vector3(x, y, 0))

    # map building
    # path finding
    # position control