#!/usr/bin/env python3

from math import radians, sin, cos
import numpy
import rospy
from duckietown_msgs.msg import Vector2D

class Transform:

    def __init__(self):

        rospy.Subscriber('/input', Vector2D, self.callback)
        self.pub_robot = rospy.Publisher('/robot', Vector2D, queue_size=10)
        self.pub_world = rospy.Publisher('/world', Vector2D, queue_size=10)
        self.new_msg = Vector2D()
             
    def callback(self, msg):

        x = msg.x    
        y = msg.y  
        
        v = [[x],[y],[1]]
        
        self.values.x = self.x_value
        self.values.y = self.y_value
        
        
        transform1 = numpy.matrix([[-1, 0, -1],[0, -1, 0],[0, 0, 1]])    
        self.calc1 = transform1 * v
        self.new_msg = self.calc1
        transform2 = numpy.matrix([[-.707, -.707, 10],[.707, -.707, 5],[0, 0, 1]])    
        self.calc2 = transform2 * calc1
        
        self.pub_robot.publish(self.new_msg)
        self.pub_world.publish(self.new_msg)

if __name__ == '__main__':
    rospy.init_node('transform', anonymous=True)
    Transform()
    
    #spin() simply keeps python from exiting until this note is stopped   
    rospy.spin()
   
