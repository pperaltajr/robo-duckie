#!/usr/bin/env python3

import rospy
import random
from std_msgs.msg import Float32
from duckietown_msgs.msg import Twist2DStamped

class Circle:
    def __init__(self):
        self.pub = rospy.Publisher('car_cmd_switch_node/cmd', Twist2DStamped, queue_size=10)
             
    def values(self, pV, pOmega):
        moveMsg = Twist2DStamped()
        moveMsg.header.stamp = rospy.get_rostime()
        moveMsg.v = pV
        moveMsg.omega = pOmega       
        self.pub.publish(moveMsg)
        		
if __name__ == '__main__':
    try:
        robot = Circle()
        rospy.init_node('Circle', anonymous=True)
        rate = rospy.Rate(0.5)
        counter = 0    
        while not rospy.is_shutdown():
            counter = counter + 1
            robot.values(0,0)
            rate.sleep()           
            if counter > 8:
                robot.values(0,0)
                break    
    except rospy.ROSInterruptException:
        rospy.loginfo("error encountered")
