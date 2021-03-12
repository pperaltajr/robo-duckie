#/user/bin/env python3

from math import radians, sin, cos
import numpy
import rospy
from duckietown_msgs.msg import Vector2D

class Transform:

    def __init__(self):

        rospy.Subscriber('/output2', Float32, self.callback)
        self.pub_units = rospy.Publisher('/robot', Float32, queue_size=10)
        self.pub_units = rospy.Publisher('/world', Float32, queue_size=10)
        self.pub_msg = UnitsLabelled()
        self.pub_msg.units = "Feet"

             
    def callback(self, msg):
        
        
   '''     self.total = msg.value
        self.pub_msg.value = self.total * 3.281
        self.pub_units.publish(self.pub_msg)
        rospy.loginfo("Converting meters from fibonacci to feet: %s", self.pub_msg)
   '''
        
if __name__ == '__main__':
    rospy.init_node('transform', anonymous=True)
    Listener()
    
    #spin() simply keeps python from exiting until this note is stopped   
    rospy.spin()
    
