#!/usr/bin/env python

from rbe500_project.srv import rrpIK

import rospy
import time
import math
import numpy

#|L0 = 0.05; L1 = 0.45; L1 = 0.425; L3 = 0.345; L4 = 0.11|
#|-------------------------------------------------------|
#|----------------TABLE OF DH PARAMETERS-----------------|
#|-------------------------------------------------------|
#|  LINK    |   THETA   |   d       |   a   |   ALPHA    |
#|   1      |   theta1  |   L0+L1   |   L2  |   0        | 
#|   2      |   theta2  |   0       |   L3  |   pi       |
#|   3      |    0      |   d3+L4   |   0   |   0        |
#|-------------------------------------------------------|

def callback_response(request):
    # Implement RRP Robot Inverse Kinematics here
    rospy.loginfo("Input Position: X = " + str(request.x_position) + " Y = " + str(request.y_position) + " Z = " + str(request.z_position))
    # z = 39/100 - d3;
    joint3 = 0.39 - request.z_position
    
    joint2 = math.acos((4000/1173) * ((request.x_position**2) + (request.y_position**2) - 0.2997))

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
