#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from math import radians, sin, cos
from odometry_hw.msg import DistWheel, Pose2D
from duckietown_msgs.msg import Twist2DStamped

class Odometry:
    def __init__(self):
        rospy.Subscriber('/dist_wheel', DistWheel, self.callback)
        self.pos = rospy.Publisher('/pose', Pose2D, queue_size=10)
        self.pos_coordinates = Pose2D()

        self.pos_coordinates.x = 0
        self.pos_coordinates.y = 0
        self.pos_coordinates.theta = 0

    def callback(self, msg):
   
        x_value = msg.dist_wheel_left
        y_value = msg.dist_wheel_right        
             
        s_delta = (x_value + y_value)/2
        theta_delta = (y_value - x_value)/(0.1)
        
        x_delta = s_delta*cos(self.pos_coordinates.theta + (theta_delta/2))
        y_delta = s_delta*sin(self.pos_coordinates.theta + (theta_delta/2))
        
        self.pos_coordinates.x = self.pos_coordinates.x + x_delta
        self.pos_coordinates.y = self.pos_coordinates.y + y_delta
        self.pos_coordinates.theta = self.pos_coordinates.theta + theta_delta    
            
        self.pos.publish(self.pos_coordinates)

       
            
            
if __name__ == '__main__':
    rospy.init_node('odometry')
    Odometry()
    
    #spin() simply keeps python from exiting until this note is stopped   
    rospy.spin()
    
