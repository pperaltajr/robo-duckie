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
        rospy.Subscriber('/image_white', Image, self.callback_white)
        rospy.Subscriber('/image_yellow', Image, self.callback_yellow)
        self.white_canny = rospy.Publisher('/white_image_canny', Image, queue_size=10)
        self.yellow_canny = rospy.Publisher('/yellow_image_canny', Image, queue_size=10)
        self.white_hough_pub = rospy.Publisher('/image_lines_white', Image, queue_size=10)
        self.yellow_hough_pub = rospy.Publisher('/image_lines_yellow', Image, queue_size=10)
        self.bridge = CvBridge()

        self.cv_img_crop = []
        self.canny = []
        self.cv_img_white = []
        self.cv_img_yellow = []
        self.white_hough = []

    def callback_crop(self, msg):

        self.cv_img_crop = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.canny = cv2.Canny(self.cv_img_crop, 120, 255)
        ros_canny = self.bridge.cv2_to_imgmsg(self.canny, "mono8")
  
    def callback_white(self, msg):
        self.cv_img_white = self.bridge.imgmsg_to_cv2(msg, "mono8")        
        if self.canny != []:
            canny_white = cv2.bitwise_and(self.canny, self.canny, mask = self.cv_img_white)
            ros_canny_white = self.bridge.cv2_to_imgmsg(canny_white, "mono8")
            self.white_canny.publish(ros_canny_white)
            self.white_hough = cv2.HoughLinesP(canny_white, 1, (np.pi/180), 10, minLineLength = 2, maxLineGap = 1)
            white_hough_lines = self.output_lines(self.cv_img_crop, self.white_hough)
            ros_white_hough = self.bridge.cv2_to_imgmsg(white_hough_lines, "bgr8")
            self.white_hough_pub.publish(ros_white_hough)
  
    def output_lines(self, original_image, lines):
        output = np.copy(original_image)
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), (255,0,0), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
                cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
        return output              
           
    def callback_yellow(self, msg):   
        self.cv_img_yellow = self.bridge.imgmsg_to_cv2(msg, "mono8")        
        if self.canny != []:
            canny_yellow = cv2.bitwise_and(self.canny, self.canny, mask = self.cv_img_yellow)
            ros_canny_yellow = self.bridge.cv2_to_imgmsg(canny_yellow, "mono8")
            self.yellow_canny.publish(ros_canny_yellow)
            self.yellow_hough = cv2.HoughLinesP(canny_yellow, 1, (np.pi/180), 7, minLineLength = 2, maxLineGap = 1)
            yellow_hough_lines = self.output_lines(self.cv_img_crop, self.yellow_hough)
            ros_yellow_hough = self.bridge.cv2_to_imgmsg(yellow_hough_lines, "bgr8")
            self.yellow_hough_pub.publish(ros_yellow_hough)   

if __name__ == '__main__':
    rospy.init_node('detect')
    detect = ImageDetect()

    #spin() simply keeps python from exiting until this note is stopped   
    rospy.spin()
