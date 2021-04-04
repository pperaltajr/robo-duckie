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
        # convert to a ROS image and crop top half
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        crop = np.array(cv_img[240:480, 0:640]) 
        
        # convert to HSV, filter for white and yellow pixels      
        image_hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        image_white = cv2.inRange(image_hsv, (0,0,215),(190,50,255))
        image_yellow = cv2.inRange(image_hsv, (0,60,220),(40,255,255))
        
        # dilation
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (4,4))
        image_dilate_white = cv2.dilate(image_white, kernel)
        image_dilate_yellow = cv2.dilate(image_yellow, kernel)
        
        # mask
        output_white = cv2.bitwise_and(crop, crop, mask=image_dilate_white)
        output_yellow = cv2.bitwise_and(crop, crop, mask=image_dilate_yellow)
        
        # convert back to original
        ros_cropped = self.bridge.cv2_to_imgmsg(crop, "bgr8")
        ros_white = self.bridge.cv2_to_imgmsg(output_white, "bgr8")
        ros_yellow = self.bridge.cv2_to_imgmsg(output_yellow, "bgr8")
        
        # published cropped, white and yellow images
        self.pub.publish(ros_cropped)
        self.white.publish(ros_white)
        self.yellow.publish(ros_yellow)

if __name__ == '__main__':
    rospy.init_node('image')
    img_crop = ImageCropper()
    
    #spin() simply keeps python from exiting until this note is stopped   
    rospy.spin()
    
