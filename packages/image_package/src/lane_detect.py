#!/usr/bin/env python3

import sys
import numpy as np
import rospy
import cv2
from duckietown_msgs.msg import Segment, SegmentList
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image

class LaneDetect:
    def __init__(self):
        rospy.Subscriber("camera_node/image/compressed", CompressedImage, self.callback, queue_size=1, buff_size=2**24)
        self.white = rospy.Publisher('/white_image_canny', Image, queue_size=10)
        self.yellow = rospy.Publisher('/yellow_image_canny', Image, queue_size=10)
        self.segment = rospy.Publisher('/lane_detector_node/segment_list', SegmentList, queue_size=10)
        self.bridge = CvBridge()

    def callback(self, msg):

        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        
        # crop image
        image_size = (160, 120)
        offset = 40
        new_img = cv2.resize(cv_img, image_size, interpolation=cv2.INTER_NEAREST)
        crop = new_img[offset:, :]
        #ros_canny = self.bridge.cv2_to_imgmsg(self.canny, "mono8")
  
        # convert to HSV, filter for white and yellow pixels      
        image_hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        image_white = cv2.inRange(image_hsv, (0,0,215),(190,50,255))
        image_yellow = cv2.inRange(image_hsv, (0,60,220),(40,255,255))
        
        # dilation
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (4,4))
        image_dilate_white = cv2.dilate(image_white, kernel)
        image_dilate_yellow = cv2.dilate(image_yellow, kernel)
        
        # find edges
        edge_image = cv2.Canny(crop, 120, 255)
        
        # while and yellow images
        edge_white = cv2.bitwise_and(edge_image, edge_image, mask=image_dilate_white)
        edge_yellow = cv2.bitwise_and(edge_image, edge_image, mask=image_dilate_yellow)
        
        
        #hough lines for white and yellow
        white_hough = cv2.HoughLinesP(edge_white, 1, (np.pi/180), 10, minLineLength = 2, maxLineGap = 1)
        white_hough_lines = self.output_lines(crop, white_hough)
        
        yellow_hough = cv2.HoughLinesP(edge_yellow, 1, (np.pi/180), 7, minLineLength = 2, maxLineGap = 1)
        yellow_hough_lines = self.output_lines(crop, yellow_hough)
        
        
        ros_white = self.bridge.cv2_to_imgmsg(white_hough_lines, "bgr8")
        ros_yellow = self.bridge.cv2_to_imgmsg(yellow_hough_lines, "bgr8")
        
                
        self.white.publish(ros_white)
        self.yellow.publish(ros_yellow)
        
        
        # Normalize
        arr_cutoff = np.array([0, offset, 0, offset])
        arr_ratio = np.array([1. / image_size[1], 1. / image_size[0], 1. / image_size[1], 1. / image_size[0]])
        yellow_normalized = (yellow_hough + arr_cutoff) * arr_ratio
        white_normalized = (white_hough + arr_cutoff) * arr_ratio
        
        segments = Segment()
        segmentList = SegmentList()
        
        for i in range(len(white_normalized)):
            segments.pixels_normalized[0].x = white_normalized[i,0,0]
            segments.pixels_normalized[0].y = white_normalized[i,0,1]
            
        for i in range(len(yellow_normalized)):
            segments.pixels_normalized[1].x = yellow_normalized[i,0,2]
            segments.pixels_normalized[1].y = yellow_normalized[i,0,3]
            
        self.segments.publish(segments)
          
    def output_lines(self, original_image, lines):
        output = np.copy(original_image)
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), (255,0,0), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
                cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
        return output
  
  
  
  
  
'''  
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
'''

if __name__ == '__main__':
    rospy.init_node('lane_detect')
    LaneDetect()
    rospy.spin()
