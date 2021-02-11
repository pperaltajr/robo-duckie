#!/usr/bin/env python3
# Adapted from http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29

import rospy
from std_msgs.msg import Float32

class Listener:
    def __init__(self):
        rospy.Subscriber('output1', Float32, self.callback)
        
    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        
if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    Listener()
    #spin() simply keeps python from exiting until this note is stopped
    
    rospy.spin()    
