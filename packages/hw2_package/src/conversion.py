#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from mystery_package.msg import UnitsLabelled

class Listener:
    def __init__(self):
#       self.total = 0
        rospy.Subscriber('/output2', UnitsLabelled, self.callback)
        self.pub_units = rospy.Publisher('/output4', UnitsLabelled, queue_size=10)
        self.pub_msg = UnitsLabelled()
        self.pub_msg.units = "Feet"

             
    def callback(self, msg):
        self.total = msg.value
        self.pub_msg.value = self.total * 3.281
        self.pub_units.publish(self.pub_msg)
        rospy.loginfo("Converting meters from fibonacci to feet: %s", self.pub_msg)
    
        
if __name__ == '__main__':
    rospy.init_node('conversion', anonymous=True)
    Listener()
    
    #spin() simply keeps python from exiting until this note is stopped   
    rospy.spin()
    
