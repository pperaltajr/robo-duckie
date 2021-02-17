#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from mystery_package.msg import UnitsLabelled

class Listener:
    def __init__(self):
        rospy.Subscriber('/output1', Float32, self.callback)
        rospy.Subscriber('/output2', UnitsLabelled, self.callback2)
   
        
    def callback(self, data):
        rospy.loginfo("I heard %s", data.data)
      
    def callback2(self, msg):
        rospy.loginfo("I heard %s", msg)
    
        
if __name__ == '__main__':
    rospy.init_node("subscriber_node")
    Listener()
    #spin() simply keeps python from exiting until this note is stopped
    
    rospy.spin()    
