#!/usr/bin/env python

from rbe500_project.srv import rrpIK, rrpIKResponse

import rospy
import time

# |------------------------------------------------------|
# |----------------TABLE OF DH PARAMETERS----------------|
# |------------------------------------------------------|
# |  LINK    |   THETA   |   d       |   a   |   ALPHA   |
# |   1      |   theta1  |   L1      |   L2  |   0       | 
# |   2      |   theta2  |   0       |   L3  |   pi      |
# |   3      |    0      |   d3+L4   |   0   |   0       |
# |------------------------------------------------------|

def callback_function(request):
    # Implement Inverse Kinematics here
    #joint1 = atan2(request.x_position, request.y_position)
    return 1

def rrp_ik_server():
    rospy.init_node("RRP_IK_service")
    service = rospy.Service("rrpIK", rrpIK, callback_function)
    rospy.spin()

if __name__ == "__main__":
    rrp_ik_server()
