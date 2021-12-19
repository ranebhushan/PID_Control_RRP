#!/usr/bin/env python

from rbe500_project.srv import rrpIK, rrpIKResponse

import rospy
import time
from math import sin, cos, tan, asin, acos,atan,atan2
import numpy

L0 = 0.05
L1 = 0.45
L2 = 0.425
L3 = 0.345
L4 = 0.11

# |L0 = 0.05; L1 = 0.45; L1 = 0.425; L3 = 0.345; L4 = 0.11|
# |-------------------------------------------------------|
# |----------------TABLE OF DH PARAMETERS-----------------|
# |-------------------------------------------------------|
# |  LINK    |   THETA   |   d       |   a   |   ALPHA    |
# |   1      |   theta1  |   L0+L1   |   L2  |   0        | 
# |   2      |   theta2  |   0       |   L3  |   pi       |
# |   3      |    0      |   d3+L4   |   0   |   0        |
# |-------------------------------------------------------|

def callback_response(request):
    # Implement RRP Robot Inverse Kinematics here
    # rospy.loginfo("Input Position: X = " + str(request.x_position) + " Y = " + str(request.y_position) + " Z = " + str(request.z_position))
    # z = 39/100 - d3;
    joint3 = (L0+L1-L4) - request.z_position
    
    joint2_cos_value = ((request.x_position**2) + (request.y_position**2) - (L2**2) - (L3**2))/(2*L2*L3)

    if joint2_cos_value > 1.0:
        joint2_cos_value = 1.0

    joint2 = acos(joint2_cos_value)
    # joint2 = acos(((request.x_position**2) + (request.y_position**2) - (L2**2) - (L3**2))/(2*L2*L3))
    
    joint1 = atan2(request.y_position, request.x_position) - atan2((L3*sin(joint2)),(L2+(L3*cos(joint2))))

    joint1 = round(joint1,4)
    joint2 = round(joint2,4)
    joint3 = round(joint3,2)

    joint1_degree = numpy.rad2deg(joint1)
    joint2_degree = numpy.rad2deg(joint2)
    joint1_degree = round(joint1_degree,4)
    joint2_degree = round(joint2_degree,4)    

    # rospy.loginfo("Joint Angles (in radians) : theta1 = " + str(joint1) + " theta2 = " + str(joint2) + " d3 = " + str(joint3))
    # rospy.loginfo("Joint Angles (in degrees) : theta1 = " + str(joint1_degree) + " theta2 = " + str(joint2_degree) + " d3 = " + str(joint3))
    return rrpIKResponse(joint1, joint2, joint3)

def rrp_ik_server():
    rospy.init_node("RRP_IK_service")
    service = rospy.Service("rrpIK", rrpIK, callback_response)
    rospy.loginfo("Ready to solve the Inverse Kinematics")
    rospy.spin()

if __name__ == "__main__":
    rrp_ik_server()
