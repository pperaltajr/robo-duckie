#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Float32
from pid_controller import PidController

class PID:
    def __init__(self):
    rospy.Subscriber("/error", Float32, self.callback)
    self.pub_control = rospy.Publisher("control_input", Float32, queue_size=10)
    self.controller = PidController()
    self.controller.Gains(0.16, 0.2, 0.8)
    
    def callback(self, data)
        error = data.data
        system_control = self.controller.calculate(error, time.time())
        self.pub.publish(system_control)
        
if __name__ == '__main__'
    rospy.init_node('pid')
    PID()
    rospy.set_param("/controller_ready", "true")
    ropsy.spin
