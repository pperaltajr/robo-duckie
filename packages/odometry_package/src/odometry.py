#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from odometry_hw.msg import DistWheel, Pose2D
from duckietown_msgs.msg import Twist2DStamped

class Odometry:
    def __init__(self):
        rospy.Subscriber('/dist_wheel', DistWheel, self.callback)
        self.pub_units = rospy.Publisher('/pose', Pose2D, queue_size=10)

             
    def callback(self, msg):
        rospy.loginfo("Pose: %s", self.pub_msg)
    
        
if __name__ == '__main__':
    rospy.init_node('odometry')
    Odometry()
    
    #spin() simply keeps python from exiting until this note is stopped   
    rospy.spin()
    
