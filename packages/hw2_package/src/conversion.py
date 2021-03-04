#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from mystery_package.msg import UnitsLabelled

class Listener:
    def __init__(self):
        rospy.Subscriber('/output2', UnitsLabelled, self.callback)
        self.pub_units = rospy.Publisher('/output4', UnitsLabelled, queue_size=10)
        self.pub_msg = UnitsLabelled()
        self.pub_msg.units = "Feet"

             
    def callback(self, msg):
        if rospy.has_param("Units"):
            self.unit = rospy.get_param("Units")
        else:
            self.unit = "Feet"
            
        # checks parameters and calculates based on units
        if self.unit == "Meters":
            self.total = msg.value
        elif self.unit == "Feet":
            self.total = self.total*3.281
        elif self.unit == "Smoots":
            self.total = self.total*1.7018

        self.pub_msg.value = self.total
        self.pub_units.publish(self.pub_msg)
        rospy.loginfo("Conversion output: %s", self.pub_msg)
    
        
if __name__ == '__main__':
    rospy.init_node('conversion', anonymous=True)
    Listener()
    
    #spin() simply keeps python from exiting until this note is stopped   
    rospy.spin()
    
