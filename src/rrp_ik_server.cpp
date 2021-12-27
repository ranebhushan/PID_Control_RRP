#include "ros/ros.h"
#include "rbe500_project/rrpIK.h"
#include <iostream>
#include <sstream>
#include <ctime>

#define _USE_MATH_DEFINES
#include <cmath>

#include <math.h>

using namespace std;

float L0 = 0.050;
float L1 = 0.450;
float L2 = 0.425;
float L3 = 0.345;
float L4 = 0.110;

// |L0 = 0.05; L1 = 0.45; L1 = 0.425; L3 = 0.345; L4 = 0.11|
// |-------------------------------------------------------|
// |----------------TABLE OF DH PARAMETERS-----------------|
// |-------------------------------------------------------|
// |  LINK    |   THETA   |   d       |   a   |   ALPHA    |
// |   1      |   theta1  |   L0+L1   |   L2  |   0        | 
// |   2      |   theta2  |   0       |   L3  |   pi       |
// |   3      |    0      |   d3+L4   |   0   |   0        |
// |-------------------------------------------------------|

/**
 * Convert the angle given in radians to degrees.
 */
template<typename F>
F rad2deg(F angle) {
    return angle * 180.0 / M_PI;
}

bool service_callback(rbe500_project::rrpIK::Request& request, rbe500_project::rrpIK::Response& response){
    // Implement RRP Robot Inverse Kinematics here
    float joint1, joint2, joint3;

    ROS_INFO("Input Position: X = %f, Y = %f, Z = %f", request.x_position, request.y_position, request.z_position);

    joint3 = (L0+L1-L4) - request.z_position;
    response.joint3 = joint3;

    float joint2_cos_value;

    joint2_cos_value = ((request.x_position*request.x_position) + (request.y_position*request.y_position) - (L2*L2) - (L3*L3))/(2*L2*L3);

    if(joint2_cos_value > 1.0){
        joint2_cos_value = 1.0;
    }

    joint2 = acos(joint2_cos_value);
    response.joint2 = joint2;

    joint1 = atan2(request.y_position, request.x_position) - atan2((L3*sin(joint2)), (L2+(L3*cos(joint2))));
    response.joint1 = joint1;

    float joint1_degree, joint2_degree;

    joint1_degree = rad2deg(joint1);
    joint2_degree = rad2deg(joint2);

    ROS_INFO("Joint Angles(in radians) : theta1 = %f, theta2 = %f, d3 = %f", joint1, joint2, joint3);

    ROS_INFO("Joint Angles(in degrees) : theta1 = %f, theta2 = %f, d3 = %f", joint1_degree, joint2_degree, joint3);

    return true;
    }

int main(int argc, char **argv){
    ros::init(argc, argv, "RRP_IK_service");
    ros::NodeHandle n;   

    ros::ServiceServer service = n.advertiseService("rrpIK", service_callback);
    ROS_INFO("Ready to solve the Inverse Kinematics");
    ros::spin();

    return 0;
}