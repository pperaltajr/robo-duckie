#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from mystery_package.msg import UnitsLabelled

class Listener:
    def __init__(self):
        rospy.Subscriber('/output2', UnitsLabelled, self.callback)
        self.pub = rospy.Publisher('/output4', UnitsLabelled, queue_size=10)
        self.pub_msg = UnitsLabelled()
        self.pub_msg.units = "feet"
             
    def callback(self, msg):
        
        if rospy.has_param("meters"):
            self.unit = rospy.get_param("meters")
            datameters = msg.value
            rospy.loginfo("%s %s", datameters, self.unit)
            self.pub.publish(datameters, self.units)
            
        elif rospy.has_param("feet"):
            self.unit = rospy.get_param("feet")
            datafeet = msg.value * 3.2808
            rospy.loginfo("%s %s", datafeet, self.unit)
            self.pub.publish(datafeet, self.units)
        
        elif rospy.has_param("smoots"):
            self.unit = rospy.get_param("smoots")
            datasmoots = msg.value * .587613
            rospy.loginfo("%s %s", datasmoots, self.unit)
            self.pub.publish(datasmoots, self.units)
            
        
if __name__ == '__main__':
    rospy.init_node('conversion', anonymous=True)
    Listener()
    
    #spin() simply keeps python from exiting until this note is stopped   
    rospy.spin()
    
