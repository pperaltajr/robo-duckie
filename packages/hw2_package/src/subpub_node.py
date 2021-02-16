#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from mystery_package.msg import UnitsLabelled

class Listener:
    def __init__(self):        
        self.total = 3.281
        self.pub_msg = UnitsLabelled()
        self.pub_msg.units = "Feet"
        rospy.Subscriber('/output2', UnitsLabelled, self.callback)
        self.pub_units = rospy.Publisher('/output3', UnitsLabelled, queue_size=10)
        
             
    def callback(self, msg):
        self.pub_msg.value = self.total
        self.pub_units.publish(self.pub_msg)
        
        
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", self.pub_msg)
    
        
if __name__ == '__main__':
    rospy.init_node('subpubnode', anonymous=True)
    Listener()
    #spin() simply keeps python from exiting until this note is stopped  
    rospy.spin()   
