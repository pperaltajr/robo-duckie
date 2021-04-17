#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from duckietown_msgs.msg import Twist2DStamped

class Straight:
    def __init__(self):
        self.pub = rospy.Publisher('car_cmd_switch_node/cmd', Twist2DStamped, queue_size=10)
             
    def move(self,y,x): 
        moveMsg = Twist2DStamped()
        moveMsg.v = y
        moveMsg.omega = x               
        self.pub.publish(moveMsg)
        		
if __name__ == '__main__':
    try:
        rospy.init_node('straight')
        robot = Straight()
        rate = rospy.Rate(10)
        counter = 0
        while not rospy.is_shutdown():
            counter = counter + 1
            robot.move(0.35,0)
            rate.sleep()           
            if counter >40:
                robot.move(0,0)
                break
        pass
    except rospy.ROSInterruptException:
        pass
