#!/usr/bin/env python

from rbe500_project.srv import rrpIK
import rospy
import time

def callback_function(request):
    # Implement Inverse Kinematics here
    return 1

def rrp_ik_server():
    rospy.init_node("RRP_IK_service")
    service = rospy.Service("rrpIK", rrpIK, callback_function)
    rospy.spin()

if __name__ == "__main__":
    rrp_ik_server()
