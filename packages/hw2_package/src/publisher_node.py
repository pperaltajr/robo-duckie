#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

class Talker:
    def __init__(self):
        self.pub = rospy.Publisher('/input', Float32, queue_size=10)
        
    def fibonacci(self):
        # Program to display the Fibonacci sequence up to the n-th term
        nterms = 8
       
        # first two terms  
        n1, n2, = 0, 1
        count = 0
       
        while count < nterms:
            print(n1)
            nth = n1 + n2
            # update values
            n1 = n2
            n2 = nth
            count += 1
            rospy.loginfo(n1)
            self.pub.publish(n1)  
        
        
if __name__ == '__main__':
    try:
        rospy.init_node("publish_node", anonymous=True)
        t = Talker()
        rate = rospy.Rate(.5) #0.5hz
        while not rospy.is_shutdown():
            t.fibonacci()
            rate.sleep()
           
            
    except rospy.ROSInterruptException:
        pass
