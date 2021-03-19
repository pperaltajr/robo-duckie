#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from math import radians, sin, cos
from odometry_hw.msg import DistWheel, Pose2D
from duckietown_msgs.msg import Twist2DStamped

class Odometry:
    def __init__(self):
        rospy.Subscriber('/dist_wheel', DistWheel, self.callback)
        self.pub = rospy.Publisher('/pose', Pose2D, queue_size=10)
        self.pub_msg = DistWheel()
        
    def callback(self, msg):
        x = 0
        y = 0
        theta = 0
        
        x_value = msg.dist_wheel_left
        y_value = msg.dist_wheel_right
        
        s_delta = (x_value + y_value)/2
        theta_delta = (x_value - y_value)/0.1
        
        x_delta = s_delta*cos(theta + theta_delta/2)
        y_delta = s_delta*sin(theta + theta_delta/2)
        
        x_prime = x + x_delta
        y_prime = y + y_delta
        theta_prime = theta + theta_delta      
        
        self.pub.publish(x_prime, y_prime, theta_prime)
    
        
if __name__ == '__main__':
    rospy.init_node('odometry')
    Odometry()
    
    #spin() simply keeps python from exiting until this note is stopped   
    rospy.spin()
    
