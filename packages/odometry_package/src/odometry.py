#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from odometry_hw.msg import DistWheel, Pose2D
from duckietown_msgs.msg import Twist2DStamped

class Odometry:
    def __init__(self):
        rospy.Subscriber('/dist_wheel', DistWheel, self.callback)
        self.pub = rospy.Publisher('/pose', Pose2D, queue_size=10)
        self.pub_msg = DistWheel()
        
    def callback(self, msg):
        x_value = msg.dist_wheel_left
        y_value = msg.dist_wheel_right
        
        
        self.pub.publish(.25, .25, .8)
    
        
if __name__ == '__main__':
    rospy.init_node('odometry')
    Odometry()
    
    #spin() simply keeps python from exiting until this note is stopped   
    rospy.spin()
    
