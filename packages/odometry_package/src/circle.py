#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from duckietown_msgs.msg import Twist2DStamped

class Circle:
    def __init__(self):
        self.pub = rospy.Publisher('car_cmd_switch_node/cmd', Twist2DStamped, queue_size=10)
             
    def move(self,y,x):
        moveMsg = Twist2DStamped()
        moveMsg.v = y
        moveMsg.omega = x       
        self.pub.publish(moveMsg)
        		
if __name__ == '__main__':
    try:
        rospy.init_node('circle')
        robot = Circle()
        rate = rospy.Rate(0.5)
        counter = 0    
        while not rospy.is_shutdown():
            counter = counter + 1
            robot.move(0.27,2.3)
            rate.sleep()           
            if counter > 4:
                robot.move(0,0)
                break    
    except rospy.ROSInterruptException:
        passros
