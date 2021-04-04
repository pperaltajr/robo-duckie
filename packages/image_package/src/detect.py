#!/usr/bin/env python3

import sys
import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class ImageDetect:
    def __init__(self):
        rospy.Subscriber('/image_cropped', Image, self.callback_crop)
        #rospy.Subscriber('/image_white', Image, self.callback_white)
        #rospy.Subscriber('/image_yellow', Image, self.callback_yellow)
        self.canny = rospy.Publisher('/image_canny', Image, queue_size=10)
        self.white = rospy.Publisher('/image_lines_white', Image, queue_size=10)
        self.yellow = rospy.Publisher('/image_lines_yellow', Image, queue_size=10)
        self.hough = rospy.Publisher('/hough', Image, queue_size=10)
        self.bridge = CvBridge()

    def callback_crop(self, msg):
        global edge
        
        # performs canny edge detection on cropped image from hw7
        cv_img_crop = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        edge = cv2.Canny(cv_img_crop, 120, 255)

        # convert to HSV, filter for white and yellow pixels      
        image_hsv = cv2.cvtColor(cv_img_crop, cv2.COLOR_BGR2HSV)
        image_white = cv2.inRange(image_hsv, (0,0,215),(190,50,255))
        image_yellow = cv2.inRange(image_hsv, (0,60,220),(40,255,255))
        
        # dilation
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (4,4))
        image_dilate_white = cv2.dilate(image_white, kernel)
        image_dilate_yellow = cv2.dilate(image_yellow, kernel)
        
        # mask
        output_white = cv2.bitwise_and(edge, edge, mask=image_dilate_white)
        output_yellow = cv2.bitwise_and(edge, edge, mask=image_dilate_yellow)
        
        # convert back to original
        ros_cropped = self.bridge.cv2_to_imgmsg(edge, "mono8")
        ros_white = self.bridge.cv2_to_imgmsg(output_white, "mono8")
        ros_yellow = self.bridge.cv2_to_imgmsg(output_yellow, "mono8") 

        
        # published cropped, white and yellow images
        self.canny.publish(ros_cropped)
        self.white.publish(ros_white)
        self.yellow.publish(ros_yellow)


        
        # hough transform
        global hough_white
        hough_white = cv2.HoughLinesP(output_white, 1, np.pi/180, 80, 20, 5)
        hough_yellow = cv2.HoughLinesP(output_yellow, 1, np.pi/180, 80, 20, 5)
        
        
        # passing output_white and hough transform to given function      
    def output_lines(self, output_white, hough_white):
        output = np.copy(output_white)
        if hough_white is not None:
            for i in range(len(hough_white)):
                l = hough_white[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), (255,0,0), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
                cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
        return output                      
        
        self.hough.publish(output_lines())
        
    '''     
    def callback_white(self, msg):
        cv_img_white = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        combine_white = cv2.bitwise_and(edge, cv_img_white)
        
        #convert back to original
        ros_white = self.bridge.cv2_to_imgmsg(combine_white, "mono8")
        
        # published cropped, white and yellow images
        self.white.publish(ros_white)
    '''
    
if __name__ == '__main__':
    rospy.init_node('detect')
    img_crop = ImageDetect()
    
    #spin() simply keeps python from exiting until this note is stopped   
    rospy.spin()
