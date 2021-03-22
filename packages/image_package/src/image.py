#!/usr/bin/env python3

import sys
import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class ImageCropper:
    def __init__(self):
        rospy.Subscriber('/image', Image, self.callback)
        self.pub = rospy.Publisher('/image_cropped', Image, queue_size=10)
        self.white = rospy.Publisher('/image_white', Image, queue_size=10)
        self.yellow = rospy.Publisher('/image_yellow', Image, queue_size=10)
        self.bridge = CvBridge()

    def callback(self, msg):
        # convert to a ROS image and crop top half, 
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        crop = np.array(cv_img[240:480, 0:640]) 
        
        # convert to HSV, filter for white and yellow pixels      
        image_hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        image_white = cv2.inRange(image_hsv, (0,0,215),(150,100,255))
        image_yellow = cv2.inRange(image_hsv, (0,0,50),(100,100,255))
        
        ros_cropped = self.bridge.cv2_to_imgmsg(crop, "bgr8")
        ros_white = self.bridge.cv2_to_imgmsg(image_white, "mono8")
        ros_yellow = self.bridge.cv2_to_imgmsg(image_yellow, "mono8")
        
        # published cropped, white and yellow images
        self.pub.publish(ros_cropped)
        self.white.publish(ros_white)
        self.yellow.publish(ros_yellow)

if __name__ == '__main__':
    rospy.init_node('image_cropper')
    img_crop = ImageCropper()
    
    #spin() simply keeps python from exiting until this note is stopped   
    rospy.spin()
    
