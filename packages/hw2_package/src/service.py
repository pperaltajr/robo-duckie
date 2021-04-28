#! /usr/bin/env python3

import time
import rospy
import actionlib
import example_action_server.msg
from std_msgs.msg import Float32
from example_service.srv import Fibonacci

def fibonacci_client(num):
    client = actionlib.SimpleActionClient('/fibonacci', example_action_server.msg.FibonacciAction)
    
    rospy.loginfo("waiting for server")
    client.wait_for_server()
    
    rospy.loginfo("sending goal")
    goal = example_action_server.msg.FibonacciGoal(order=num)
    client.send_goal(goal)
    rospy.loginfo("goal sent")
    
    client.wait_for_result()
    rospy.loginfo("goal received")
    
    return client.get_result()
    
def fibcalc(num):
    rospy.loginfo("waiting for service")
    rospy.wait_for_service("/calc_fibonacci")
    try:
        fibcalc = rospy.ServiceProxy("/calc_fibonacci", Fibonacci)
        rospy.loginfo("service requested")
        answer = fibcalc(num)
        rospy.loginfo("result received")
        print(answer)
        return answer
        
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))
        rospy.loginfo("Service Error")
        
    
 

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        time.sleep(3)
        rospy.init_node('fibonacci_client_py')
        fibcalc(3)
        result = fibonacci_client(3)
        print("Result:", ', '.join([str(n) for n in result.sequence]))
        
        fibcalc(15)
        result = fibonacci_client(15)
        print("Result:", ', '.join([str(n) for n in result.sequence]))
        
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
