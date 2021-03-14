#!/usr/bin/env python3

import rospy
import random
from time import sleep
from duckietown_msgs.msg import Twist2DStamped

class Line:

    def __init__(self):

        self.pub = rospy.Publisher('car_cmd_switch_node/cmd', Twist2DStamped, queue_size=10)
             
    def straight(self):
        counter = 0
        rate = rospy.Rate(0.5)
        
        moveMsg = Twist2DStamped()
        moveMsg.header.stamp = rospy.get_rostime()
        moveMsg.v = 0.234
        moveMsg.omega = 0.1
        
        self.pub.publish(moveMsg)
        
'''        while not rospy.is_shutdown():
            self.pub.publish(moveMsg)
            counter = counter + 1
            rate.sleep()
            
            if counter > 1:
                robot.send(0,0)
'''
        		
if __name__ == '__main__':
    try:
        robot = Line()
        rospy.init_node('line', anonymous=True)
        robot.straight()
    
    except rospy.ROSInterruptException:
        rospy.loginfo("error encountered")
