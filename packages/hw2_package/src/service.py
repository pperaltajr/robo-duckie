#!/usr/bin/env python3

import rospy
from beginner_tutorials.srv import*
    
def add_two_ints_client(x,y):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        resp1 = add_two_ints(x,y)
        return resp1.sum
    except rospy.ServiceException, e:
        print("Service call failed: %s" %e)


if __name__ == '__main__':
    add_two_ints_client(1,2)
