#/user/bin/env python3

from math import radians, sin, cos
import numpy
import rospy
from duckietown_msgs.msg import Vector2D

class Transform:

    def __init__(self):

        rospy.Subscriber('/output2', Vector2D, self.callback)
        self.pub_robot = rospy.Publisher('/robot', Vector2D, queue_size=10)
        self.pub_world = rospy.Publisher('/world', Vector2D, queue_size=10)
        self.values = Vector2D()
        x = self.values.x
        y = self.values.y
             
    def callback(self, msg):
            
        v = [[x],[y],[1]]
        transform1 = numpy.matrix([[-1, 0, -1],[0, -1, 0],[0, 0, 1]])    
        calc1 = transform * v
        
        transform2 = numpy.matrix([[-.707, -.707, 10],[.707, -.707, 5],[0, 0, 1]])    
        calc2 = transform * v
        
        self.pub_robot.publish(calc1)
        self.pub_world.publish(calc2)
        
        
if __name__ == '__main__':
    rospy.init_node('transform', anonymous=True)
    Transform()
    
    #spin() simply keeps python from exiting until this note is stopped   
    rospy.spin()
   
