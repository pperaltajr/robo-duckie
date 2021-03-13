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
        x_value = msg.x    
        y_value = msg.y         
        rps = numpy.matrix([[x_value],[y_value],[1]])

        rts = numpy.matrix([[-1, 0, -1],[0, -1, 0],[0, 0, 1]])    
        transform = rts * rps
        self.pub_robot.publish(transform[0,0], transform[1,0])
        
        wts = numpy.matrix([[-.707, -.707, 10],[.707, -.707, 5],[0, 0, 1]])    
        transform2 = wts * transform
        self.pub_world.publish(transform2[0,0], transform2[1,0])
        		
if __name__ == '__main__':
    rospy.init_node('transform', anonymous=True)
    Transform()
    
    #spin() simply keeps python from exiting until this note is stopped   
    rospy.spin()
