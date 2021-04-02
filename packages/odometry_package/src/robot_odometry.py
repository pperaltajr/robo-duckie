#!/usr/bin/env python3

import rospy
from math import radians, sin, cos
from duckietown_msgs.msg import WheelsCmdStamped, Pose2DStamped

class OdometryRobot:
    def __init__(self):
        rospy.Subscriber('/wheels_driver_node/wheels_cmd', WheelsCmdStamped, self.callback)
        self.pos = rospy.Publisher('/robot_pose', Pose2DStamped, queue_size=10)
        self.pos_coordinates = Pose2DStamped()
        
        # sets initial coordinates to zero
        self.pos_coordinates.x = 0
        self.pos_coordinates.y = 0
        self.pos_coordinates.theta = 0

    def callback(self, msg):
        # pulls distance from left and right wheel
        x_value = msg.dist_left
        y_value = msg.dist_right        
        
        # performs calculations for odometry formula    
        s_delta = (x_value + y_value)/2
        theta_delta = (y_value - x_value)/(0.1)
        
        x_delta = s_delta*cos(self.pos_coordinates.theta + (theta_delta/2))
        y_delta = s_delta*sin(self.pos_coordinates.theta + (theta_delta/2))
        
        # adds previous coordinates with the delta change and reassigns values
        self.pos_coordinates.x = self.pos_coordinates.x + x_delta
        self.pos_coordinates.y = self.pos_coordinates.y + y_delta
        self.pos_coordinates.theta = self.pos_coordinates.theta + theta_delta    
           
        self.pos.publish(self.pos_coordinates)         
            
if __name__ == '__main__':
    rospy.init_node('odometry_robot')
    OdometryRobot()
    
    #spin() simply keeps python from exiting until this note is stopped   
    rospy.spin()
    
