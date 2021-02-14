#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from mystery_package.msg import UnitsLabelled

class Listener:
    def __init__(self):
        rospy.Subscriber('/output2', UnitsLabelled, self.callback2)
        self.pub = rospy.Publisher('/input', Float32, queue_size=10)
        
             
    def callback2(self, msg):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg)
    
        
if __name__ == '__main__':
    rospy.init_node('subpubnode', anonymous=True)
    Listener()
    #spin() simply keeps python from exiting until this note is stopped
    
    rospy.spin()   
