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
        self.Segments = rospy.Publisher('/line_detector_node/segment_list', SegmentList, queue_size=10) 
        self.white = rospy.Publisher('/white_image', Image, queue_size=10)
        self.yellow = rospy.Publisher('/yellow_image', Image, queue_size=10)   
        self.hough = rospy.Publisher('/hough_lines', Image, queue_size=10) 
        self.bridge = CvBridge()
        self.loop = 0
        
    def output_lines(self, original_image, lines):
        output = np.copy(original_image)
        if lines is not None:
            for i in range(len(lines)):
                l = lines[i][0]
                cv2.line(output, (l[0],l[1]), (l[2],l[3]), (255,0,0), 2, cv2.LINE_AA)
                cv2.circle(output, (l[0],l[1]), 2, (0,255,0))
                cv2.circle(output, (l[2],l[3]), 2, (0,0,255))
        return output
        
    def callback(self, msg):   
        rospy.loginfo("DETECTION IN PROGRESS") 
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        
        # crop image
        image_size = (160, 120)
        offset = 40
        new_img = cv2.resize(cv_img, image_size, interpolation=cv2.INTER_NEAREST)
        crop = new_img[offset:, :]        
        #crop_output = self.bridge.cv2_to_imgmsg(crop, "bgr8")
        #self.crop.publish(crop_output)
        
  
        # convert to HSV, filter for white and yellow pixels      
        image_hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        image_white = cv2.inRange(image_hsv, (0,0,67),(170,70,255))
        image_yellow = cv2.inRange(image_hsv, (0,60,110),(40,255,255))
        
        # dilation
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (4,4))
        image_dilate_white = cv2.dilate(image_white, kernel)
        image_dilate_yellow = cv2.dilate(image_yellow, kernel)
               
        # find edges
        edge_image = cv2.Canny(crop, 120, 255)
        
        # while and yellow images
        edge_white = cv2.bitwise_and(edge_image, edge_image, mask=image_dilate_white)
        edge_yellow = cv2.bitwise_and(edge_image, edge_image, mask=image_dilate_yellow)
        
        #white_output = self.bridge.cv2_to_imgmsg(edge_white, "bgr8")
        #yellow_output = self.bridge.cv2_to_imgmsg(edge_yellow, "bgr8")        
        #self.white.publish(white_output)
        #self.yellow.publish(yellow_output)       
                      
        #hough lines for white and yellow
        white_hough = cv2.HoughLinesP(edge_white, 1, (np.pi/180), 10, minLineLength = 2, maxLineGap = 1)
        white_hough_lines = self.output_lines(crop, white_hough)
        
        yellow_hough = cv2.HoughLinesP(edge_yellow, 1, (np.pi/180), 7, minLineLength = 2, maxLineGap = 1)
        yellow_hough_lines = self.output_lines(crop, yellow_hough)
        
        
        ros_white = self.bridge.cv2_to_imgmsg(white_hough_lines, "bgr8")
        ros_yellow = self.bridge.cv2_to_imgmsg(yellow_hough_lines, "bgr8")
        
                
        self.white.publish(ros_white)
        self.yellow.publish(ros_yellow)
        
        
        hough_combined = self.output_lines(yellow_hough_lines, white_hough)
        ros_hough_combined = self.bridge.cv2_to_imgmsg(hough_combined, "bgr8")
        self.hough.publish(ros_hough_combined)
       
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
                    
        #self.Segments.publish(segments)

  

if __name__ == '__main__':
    rospy.init_node('lane_detect')
    LaneDetect()
    rospy.spin()

    
