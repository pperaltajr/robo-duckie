#!/usr/bin/env python3

import rospy
import random
import time
from std_msgs.msg import Float32
from duckietown_msgs.msg import Twist2DStamped, LanePose
from pid_controller import PidController

class LaneController:
    def __init__(self):
        rospy.Subscriber("lane_filter_node/lane_pose", LanePose, self.CalculateCarCmd)
        self.lane_controller = rospy.Publisher("lane_controller_node/car_cmd", Twist2DStamped, queue_size=10)
        self.d_controller = PidController()
        self.phi_controller = PidController()
        
        self.d_controller.UpdateGains(7, 0.02, 0)
        self.phi_controller.UpdateGains(7, 0.04, 0)

    def SendCarCmd(self, pV, pOmega):
        moveMsg = Twist2DStamped()
        moveMsg.header.stamp = rospy.get_rostime()
        moveMsg.v = pV
        moveMsg.omega = pOmega
        self.lane_controller.publish(moveMsg)
        
    def CalculateCarCmd(self, data):
        d_error = 0 - data.d
        phi_error = 0 -data.phi
        
        d_control = self.d_controller.calculate(d_error, time.time())
        phi_control = self.phi_controller.calculate(phi_error, time.time())
        
        omega = d_control + phi_control
        
        self.SendCarCmd(0.2, omega)
        
if __name__ == '__main__':      
    rospy.init_node('lab3_lane_controller_node', anonymous=True)
    LaneController()
    rospy.spin()
