#!/usr/bin/env python

from rbe500_project.srv import rrpIK

import rospy
import time
import math
import numpy

# |------------------------------------------------------|
# |----------------TABLE OF DH PARAMETERS----------------|
# |------------------------------------------------------|
# |  LINK    |   THETA   |   d       |   a   |   ALPHA   |
# |   1      |   theta1  |   L1      |   L2  |   0       | 
# |   2      |   theta2  |   0       |   L3  |   pi      |
# |   3      |    0      |   d3+L4   |   0   |   0       |
# |------------------------------------------------------|

def callback_response(request):
    # Implement RRP Robot Inverse Kinematics here
    rospy.loginfo("Input Position: X = " + str(request.x_position) + " Y = " + str(request.y_position) + " Z = " + str(request.z_position))
    joint1 = math.atan2(request.y_position, request.x_position)
  
    # Testing Purpose
    joint2 = request.y_position + 2 
    joint3 = request.z_position + 3
    rospy.loginfo("Joint Angles (in radians) : theta1 = " + str(joint1) + " theta2 = " + str(joint2) + " d3 = " + str(joint3))
    rospy.loginfo("Joint Angles (in degrees ) : theta1 = " + str(numpy.rad2deg(joint1)) + " theta2 = " + str(numpy.rad2deg(joint2)) + " d3 = " + str(joint3))
    return (joint1, joint2, joint3)

def rrp_ik_server():
    rospy.init_node("RRP_IK_service")
    service = rospy.Service("rrpIK", rrpIK, callback_response)
    rospy.loginfo("Ready to solve the Inverse Kinematics")
    rospy.spin()

if __name__ == "__main__":
    rrp_ik_server()
