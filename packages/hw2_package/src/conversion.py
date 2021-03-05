#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from mystery_package.msg import UnitsLabelled

class Listener:
    def __init__(self):
        rospy.Subscriber('/output2', UnitsLabelled, self.callback)
        self.pub_units = rospy.Publisher('/output4', UnitsLabelled, queue_size=10)
        self.pub_msg = UnitsLabelled()
        self.pub_msg.units = "feet"
             
    def callback(self, msg):
        if rospy.has_param("units"):
            self.units = rospy.get_param("units")
        else:
            self.units = "no parameter"
        
        if self.units == "meters":
            self.pub_msg.units = "meters"
            self.total = msg.value
            self.pub_msg.value = self.total
            self.pub_units.publish(self.pub_msg)
            rospy.loginfo("No conversion for meters: %s", self.pub_msg)      
            
        elif self.units == "feet":
            self.pub_msg.units = "feet"
            self.total = msg.value * 3.2808
            self.pub_msg.value = self.total
            self.pub_units.publish(self.pub_msg)
            rospy.loginfo("Conversion for feet: %s", self.pub_msg) 
            
        elif self.units == "smoots":
            self.pub_msg.units = "smoots"
            self.total = msg.value * 0.587613
            self.pub_msg.value = self.total
            self.pub_units.publish(self.pub_msg)
            rospy.loginfo("Conversion for smoots: %s", self.pub_msg)       
        
            #rospy.loginfo("value: %s units: %s, pub__msg.value, pub_msg.units)      
     
        
if __name__ == '__main__':
    rospy.init_node('conversion', anonymous=True)
    Listener()
    
    #spin() simply keeps python from exiting until this note is stopped   
    rospy.spin()
    
    
    
   
