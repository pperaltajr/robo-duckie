#!/usr/bin/env python3

from beginner_tutorials.srv import AddTwoInts, AddTwoIntsResponse
import rospy

def hangle_add_two_ints(rep):
    return AddTwoIntsResponse(req.a +req.b)
    
def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    
if __name__ == '__main__':
    add_two_ints_server()
    rospy.spin()
