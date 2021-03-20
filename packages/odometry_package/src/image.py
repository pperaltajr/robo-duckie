#!/usr/bin/env python3

import sys
import rospy
import cv2
sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Image:
    def __init__(self):
        rospy.Subscriber('/image', Image, self.callback)
        self.pub = rospy.Publisher('/image_cropped', Image, queue_size=10)
        self.white = rospy.Publisher('/image_white', Image, queue_size=10)
        self.yellow = rospy.Publisher('/image_yellow', Image, queue_size=10)
        self.bridge = CvBridge()


    def callback(self, msg):

if __name__ == '__main__':
    rospy.init_node('image_cropper')
    Image()
    
    #spin() simply keeps python from exiting until this note is stopped   
    rospy.spin()
    
