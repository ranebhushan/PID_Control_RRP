#!/usr/bin/env python

import rospy
from time import sleep
from math import pi
import numpy 
from std_msgs.msg import String, Float64, Float64MultiArray, Header, _String
from sensor_msgs.msg import JointState
from rbe500_project.srv import rrpIK

# Desired end-effector positions
p1 = [0, 0.77, 0.34]
p2 = [-0.345, 0.425, 0.24]
p3 = [-0.67, -0.245, 0.14]
p4 = [0.77, 0, 0.44]

x = 0
y = 1
z = 2

class PID_Controller():

    def __init__(self, P = 0.0, I = 0.0, D = 0.0, set_point=0):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.set_point = set_point # reference (desired joint position)
        self.error_integral = 0
        self.previous_error = 0
        
    def update(self, current_value):
        # calculate P_term and D_term
        error = self.set_point - current_value
        P_term = self.Kp*error
        self.error_integral = self.error_integral + error
        I_term = self.Ki*(self.error_integral)
        D_term = self.Kd*(error - self.previous_error)
        self.previous_error = error
        return P_term + I_term + D_term

    def setPoint(self, set_point):
        self.set_point = set_point
        self.previous_error = 0
    
    def setPID(self, P = 1.0, I = 0.0, D = 0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D

class RRP_robot():

    def __init__(self):

        rospy.init_node("rrp_robot_move")

        rospy.loginfo("Press Ctrl + C to terminate")

        # Publish Joint Velocity to Joint1 Effort Controller
        self.joint1_velocity = Float64()
        self.joint1_velocity.data = 0
        self.joint1_velocity_publisher = rospy.Publisher("/rrp/joint1_effort_controller/command", Float64, queue_size=10)

        # Publish Joint Velocity to Joint2 Effort Controller
        self.joint2_velocity = Float64()
        self.joint2_velocity.data = 0
        self.joint2_velocity_publisher = rospy.Publisher("/rrp/joint2_effort_controller/command", Float64, queue_size=10)

        # Publish Joint Velocity to Joint3 Effort Controller
        self.joint3_velocity = Float64()
        self.joint3_velocity.data = 0
        self.joint3_velocity_publisher = rospy.Publisher("/rrp/joint3_effort_controller/command", Float64, queue_size=10)

        self.rate = rospy.Rate(10)

        # Subscribe to Joint States Topic
        self.current_value = JointState()
        self.data_log_counter = 0
        self.robot_path = list()
        self.joint_state_subscriber = rospy.Subscriber("/rrp/joint_states", JointState, self.joint_state_callback)
   
        try:
            self.robot_move()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")

    def joint_state_callback(self, msg):
        # Get current joint angles [joint1, joint2, joint3] from /rrp/joint_states topic
        # Get current joint veloctiy [w1, w2, v3] from /rrp/joint_states topic

        self.joint1 = msg.position[0]
        self.joint2 = msg.position[1]
        self.joint3 = msg.position[2]

        self.joint1_velocity = msg.velocity[0]
        self.joint2_velocity = msg.velocity[1]
        self.joint3_velocity = msg.velocity[2]

    def robot_move(self):
        joint1_PID_controller = PID_Controller(P = 1.0, I = 0.0, D = 0.0, set_point = 0)
        joint2_PID_controller = PID_Controller(P = 1.0, I = 0.0, D = 0.0, set_point = 0)
        joint3_PID_controller = PID_Controller(P = 1.0, I = 0.0, D = 0.0, set_point = 0)   
        rospy.wait_for_service('rrpIK', timeout=None)
        get_joint_variables = rospy.ServiceProxy('rrpIK', rrpIK)
        rospy.loginfo("Requesting Joint Variables for end-effector position p1...")
        rospy.loginfo("X = " + str(p1[x]) + " Y = " + str(p1[y]) + " Z = " + str(p1[z]))
        data = get_joint_variables(p1[x], p1[y], p1[z])
        

if __name__ == '__main__':
    pid_controller = PID_Controller()
    rrp_robot = RRP_robot()