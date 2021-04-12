#!/usr/bin/env python3

import rospy
from math import radians, sin, cos
from duckietown_msgs.msg import WheelsCmdStamped, Pose2DStamped

class OdometryRobot:
    def __init__(self):
        rospy.Subscriber('wheels_driver_node/wheels_cmd', WheelsCmdStamped, self.callback)
        self.pos = rospy.Publisher('robot_pose', Pose2DStamped, queue_size=1)
        self.pos_coordinates = Pose2DStamped()
        
        # sets initial coordinates to zero
        self.pos_coordinates.x = 0
        self.pos_coordinates.y = 0
        self.pos_coordinates.theta = 0
        time = rospy.get_rostime()
        self.past_time = time.secs + (time.nsecs/1000000000)
        
    def callback(self, msg):
        time = rospy.get_rostime()
        current_time = time.secs + (time.nsecs/1000000000)
        time_diff = current_time - self.past_time
        self.past_time = current_time
        if time_diff>1:
            return
        trim = .91
        # pulls distance from left and right wheel
        x_value = msg.vel_left * time_diff
        y_value = msg.vel_right * time_diff * trim
        
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
