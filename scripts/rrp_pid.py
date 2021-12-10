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
p4 = [0.77, 0.0, 0.39]

x = 0
y = 1
z = 2

JOINT1_EFFORT_LIMIT = 1.0
JOINT1_TOLERANCE = 0.05

JOINT2_EFFORT_LIMIT = 1.0
JOINT2_TOLERANCE = 0.05

JOINT3_EFFORT_LIMIT = 1.0
JOINT3_TOLERANCE = 0.05

class PID_Controller():

    def __init__(self, P = 0.0, I = 0.0, D = 0.0, set_point=0):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.set_point = set_point # reference (desired joint position)
        self.error_integral = 0
        self.previous_error = 0
        
    def update(self, current_value):
        # calculate P_term, I_term and D_term
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
        self.joint1_effort = Float64()
        self.joint1_effort.data = 0
        self.joint1_effort_publisher = rospy.Publisher("/rrp/joint1_effort_controller/command", Float64, queue_size=10)

        # Publish Joint Velocity to Joint2 Effort Controller
        self.joint2_effort = Float64()
        self.joint2_effort.data = 0
        self.joint2_effort_publisher = rospy.Publisher("/rrp/joint2_effort_controller/command", Float64, queue_size=10)

        # Publish Joint Velocity to Joint3 Effort Controller
        self.joint3_effort = Float64()
        self.joint3_effort.data = 0
        self.joint3_effort_publisher = rospy.Publisher("/rrp/joint3_effort_controller/command", Float64, queue_size=10)

        self.rate = rospy.Rate(10)

        # Subscribe to Joint States Topic
        self.current_value = JointState()
        self.data_log_counter = 0
        self.robot_path = list()
        self.joint_state_subscriber = rospy.Subscriber("/rrp/joint_states", JointState, self.joint_state_callback)

        self.joint1_flag = 0
        self.joint2_flag = 0
        self.joint3_flag = 0

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
        
        # Define Joint Efforts for Robot to stay at origin
        self.joint1_effort = 0
        self.joint2_effort = 0
        self.joint3_effort = 0
        self.joint1_effort_publisher.publish(self.joint1_effort)
        self.joint2_effort_publisher.publish(self.joint2_effort)
        self.joint3_effort_publisher.publish(self.joint3_effort)

        # Get joint angles corresponding to position p1(0, 0.77, 0.34)
        rospy.loginfo("Requesting Joint Variables for Position p1")
        rospy.loginfo("X = " + str(p1[x]) + " Y = " + str(p1[y]) + " Z = " + str(p1[z]))
        p1_joint_variables = get_joint_variables(p1[x], p1[y], p1[z])
        
        joint1_PID_controller.setPoint(p1_joint_variables.joint1)
        joint2_PID_controller.setPoint(p1_joint_variables.joint2)
        joint3_PID_controller.setPoint(p1_joint_variables.joint3)

        while (self.joint1_flag == 0 or self.joint2_flag == 0 or self.joint3_flag == 0):
            self.joint1_effort = joint1_PID_controller.update(self.joint1)
            self.joint2_effort = joint2_PID_controller.update(self.joint2)
            self.joint3_effort = joint3_PID_controller.update(self.joint3)
            rospy.loginfo("Joint1 = " + str(self.joint1) + " Joint2 = " + str(self.joint2) + " Joint3 = " + str(self.joint3))
            # Effort Constraints
            if (self.joint1_effort > JOINT1_EFFORT_LIMIT):
                self.joint1_effort = JOINT1_EFFORT_LIMIT
            elif (self.joint1_effort < -JOINT1_EFFORT_LIMIT):
                self.joint1_effort = -JOINT1_EFFORT_LIMIT
            
            if (self.joint2_effort > JOINT2_EFFORT_LIMIT):
                self.joint2_effort = JOINT2_EFFORT_LIMIT
            elif (self.joint2_effort < -JOINT2_EFFORT_LIMIT):
                self.joint2_effort = -JOINT2_EFFORT_LIMIT

            if (self.joint3_effort > JOINT3_EFFORT_LIMIT):
                self.joint3_effort = JOINT3_EFFORT_LIMIT
            elif (self.joint3_effort < -JOINT3_EFFORT_LIMIT):
                self.joint3_effort = -JOINT3_EFFORT_LIMIT
            
            # Loop Break Condition
            if (self.joint1 > (p1_joint_variables.joint1 - JOINT1_TOLERANCE) and self.joint1 < (p1_joint_variables.joint1 + JOINT1_TOLERANCE)):
                self.joint1_flag = 1
            if (self.joint2 > (p1_joint_variables.joint2 - JOINT2_TOLERANCE) and self.joint2 < (p1_joint_variables.joint2 + JOINT2_TOLERANCE)):
                self.joint2_flag = 1
            if (self.joint3 > (p1_joint_variables.joint3 - JOINT3_TOLERANCE) and self.joint3 < (p1_joint_variables.joint3 + JOINT3_TOLERANCE)):
                self.joint3_flag = 1
            # Publish effort from PID controller to Robot 
            self.joint1_effort_publisher.publish(self.joint1_effort)
            self.joint2_effort_publisher.publish(self.joint2_effort)
            self.joint3_effort_publisher.publish(self.joint3_effort)

        # Stop the Robot
        self.joint1_effort = 0
        self.joint2_effort = 0
        self.joint3_effort = 0
        self.joint1_effort_publisher.publish(self.joint1_effort)
        self.joint2_effort_publisher.publish(self.joint2_effort)
        self.joint3_effort_publisher.publish(self.joint3_effort)
        self.joint1_flag = 0
        self.joint2_flag = 0
        self.joint3_flag = 0

        # Stop for 1 second at each desired configuration
        sleep(1)

        # Get joint angles corresponding to position p2(-0.345, 0.425, 0.24)
        rospy.loginfo("Requesting Joint Variables for Position p2")
        rospy.loginfo("X = " + str(p2[x]) + " Y = " + str(p2[y]) + " Z = " + str(p2[z]))
        p2_joint_variables = get_joint_variables(p2[x], p2[y], p2[z])
        
        joint1_PID_controller.setPoint(p2_joint_variables.joint1)
        joint2_PID_controller.setPoint(p2_joint_variables.joint2)
        joint3_PID_controller.setPoint(p2_joint_variables.joint3)

        while (self.joint1_flag == 0 or self.joint2_flag == 0 or self.joint3_flag == 0):
            self.joint1_effort = joint1_PID_controller.update(self.joint1)
            self.joint2_effort = joint2_PID_controller.update(self.joint2)
            self.joint3_effort = joint3_PID_controller.update(self.joint3)
            rospy.loginfo("Joint1 = " + str(self.joint1) + " Joint2 = " + str(self.joint2) + " Joint3 = " + str(self.joint3))
            # Effort Constraints
            if (self.joint1_effort > JOINT1_EFFORT_LIMIT):
                self.joint1_effort = JOINT1_EFFORT_LIMIT
            elif (self.joint1_effort < -JOINT1_EFFORT_LIMIT):
                self.joint1_effort = -JOINT1_EFFORT_LIMIT
            
            if (self.joint2_effort > JOINT2_EFFORT_LIMIT):
                self.joint2_effort = JOINT2_EFFORT_LIMIT
            elif (self.joint2_effort < -JOINT2_EFFORT_LIMIT):
                self.joint2_effort = -JOINT2_EFFORT_LIMIT

            if (self.joint3_effort > JOINT3_EFFORT_LIMIT):
                self.joint3_effort = JOINT3_EFFORT_LIMIT
            elif (self.joint3_effort < -JOINT3_EFFORT_LIMIT):
                self.joint3_effort = -JOINT3_EFFORT_LIMIT
            
            # Loop Break Condition
            if (self.joint1 > (p2_joint_variables.joint1 - JOINT1_TOLERANCE) and self.joint1 < (p2_joint_variables.joint1 + JOINT1_TOLERANCE)):
                self.joint1_flag = 1
            if (self.joint2 > (p2_joint_variables.joint2 - JOINT2_TOLERANCE) and self.joint2 < (p2_joint_variables.joint2 + JOINT2_TOLERANCE)):
                self.joint2_flag = 1
            if (self.joint3 > (p2_joint_variables.joint3 - JOINT3_TOLERANCE) and self.joint3 < (p2_joint_variables.joint3 + JOINT3_TOLERANCE)):
                self.joint3_flag = 1
            # Publish effort from PID controller to Robot 
            self.joint1_effort_publisher.publish(self.joint1_effort)
            self.joint2_effort_publisher.publish(self.joint2_effort)
            self.joint3_effort_publisher.publish(self.joint3_effort)

        # Stop the Robot
        self.joint1_effort = 0
        self.joint2_effort = 0
        self.joint3_effort = 0
        self.joint1_effort_publisher.publish(self.joint1_effort)
        self.joint2_effort_publisher.publish(self.joint2_effort)
        self.joint3_effort_publisher.publish(self.joint3_effort)
        self.joint1_flag = 0
        self.joint2_flag = 0
        self.joint3_flag = 0

        # Stop for 1 second at each desired configuration
        sleep(1)

        # Get joint angles corresponding to position p3(-0.67, -0.245, 0.14)
        rospy.loginfo("Requesting Joint Variables for Position p3")
        rospy.loginfo("X = " + str(p3[x]) + " Y = " + str(p3[y]) + " Z = " + str(p3[z]))
        p3_joint_variables = get_joint_variables(p3[x], p3[y], p3[z])
        
        joint1_PID_controller.setPoint(p3_joint_variables.joint1)
        joint2_PID_controller.setPoint(p3_joint_variables.joint2)
        joint3_PID_controller.setPoint(p3_joint_variables.joint3)

        while (self.joint1_flag == 0 or self.joint2_flag == 0 or self.joint3_flag == 0):
            self.joint1_effort = joint1_PID_controller.update(self.joint1)
            self.joint2_effort = joint2_PID_controller.update(self.joint2)
            self.joint3_effort = joint3_PID_controller.update(self.joint3)
            rospy.loginfo("Joint1 = " + str(self.joint1) + " Joint2 = " + str(self.joint2) + " Joint3 = " + str(self.joint3))
            # Effort Constraints
            if (self.joint1_effort > JOINT1_EFFORT_LIMIT):
                self.joint1_effort = JOINT1_EFFORT_LIMIT
            elif (self.joint1_effort < -JOINT1_EFFORT_LIMIT):
                self.joint1_effort = -JOINT1_EFFORT_LIMIT
            
            if (self.joint2_effort > JOINT2_EFFORT_LIMIT):
                self.joint2_effort = JOINT2_EFFORT_LIMIT
            elif (self.joint2_effort < -JOINT2_EFFORT_LIMIT):
                self.joint2_effort = -JOINT2_EFFORT_LIMIT

            if (self.joint3_effort > JOINT3_EFFORT_LIMIT):
                self.joint3_effort = JOINT3_EFFORT_LIMIT
            elif (self.joint3_effort < -JOINT3_EFFORT_LIMIT):
                self.joint3_effort = -JOINT3_EFFORT_LIMIT
            
            # Loop Break Condition
            if (self.joint1 > (p3_joint_variables.joint1 - JOINT1_TOLERANCE) and self.joint1 < (p3_joint_variables.joint1 + JOINT1_TOLERANCE)):
                self.joint1_flag = 1
            if (self.joint2 > (p3_joint_variables.joint2 - JOINT2_TOLERANCE) and self.joint2 < (p3_joint_variables.joint2 + JOINT2_TOLERANCE)):
                self.joint2_flag = 1
            if (self.joint3 > (p3_joint_variables.joint3 - JOINT3_TOLERANCE) and self.joint3 < (p3_joint_variables.joint3 + JOINT3_TOLERANCE)):
                self.joint3_flag = 1
            # Publish effort from PID controller to Robot 
            self.joint1_effort_publisher.publish(self.joint1_effort)
            self.joint2_effort_publisher.publish(self.joint2_effort)
            self.joint3_effort_publisher.publish(self.joint3_effort)

        # Stop the Robot
        self.joint1_effort = 0
        self.joint2_effort = 0
        self.joint3_effort = 0
        self.joint1_effort_publisher.publish(self.joint1_effort)
        self.joint2_effort_publisher.publish(self.joint2_effort)
        self.joint3_effort_publisher.publish(self.joint3_effort)
        self.joint1_flag = 0
        self.joint2_flag = 0
        self.joint3_flag = 0

        # Stop for 1 second at each desired configuration
        sleep(1)

        # Get joint angles corresponding to position p4(0.77, 0, 0.44)
        rospy.loginfo("Requesting Joint Variables for Position p4")
        rospy.loginfo("X = " + str(p4[x]) + " Y = " + str(p4[y]) + " Z = " + str(p4[z]))
        p4_joint_variables = get_joint_variables(p4[x], p4[y], p4[z])
        
        joint1_PID_controller.setPoint(p4_joint_variables.joint1)
        joint2_PID_controller.setPoint(p4_joint_variables.joint2)
        joint3_PID_controller.setPoint(p4_joint_variables.joint3)

        while (self.joint1_flag == 0 or self.joint2_flag == 0 or self.joint3_flag == 0):
            self.joint1_effort = joint1_PID_controller.update(self.joint1)
            self.joint2_effort = joint2_PID_controller.update(self.joint2)
            self.joint3_effort = joint3_PID_controller.update(self.joint3)
            rospy.loginfo("Joint1 = " + str(self.joint1) + " Joint2 = " + str(self.joint2) + " Joint3 = " + str(self.joint3))
            # Effort Constraints
            if (self.joint1_effort > JOINT1_EFFORT_LIMIT):
                self.joint1_effort = JOINT1_EFFORT_LIMIT
            elif (self.joint1_effort < -JOINT1_EFFORT_LIMIT):
                self.joint1_effort = -JOINT1_EFFORT_LIMIT
            
            if (self.joint2_effort > JOINT2_EFFORT_LIMIT):
                self.joint2_effort = JOINT2_EFFORT_LIMIT
            elif (self.joint2_effort < -JOINT2_EFFORT_LIMIT):
                self.joint2_effort = -JOINT2_EFFORT_LIMIT

            if (self.joint3_effort > JOINT3_EFFORT_LIMIT):
                self.joint3_effort = JOINT3_EFFORT_LIMIT
            elif (self.joint3_effort < -JOINT3_EFFORT_LIMIT):
                self.joint3_effort = -JOINT3_EFFORT_LIMIT
            
            # Loop Break Condition
            if (self.joint1 > (p4_joint_variables.joint1 - JOINT1_TOLERANCE) and self.joint1 < (p4_joint_variables.joint1 + JOINT1_TOLERANCE)):
                self.joint1_flag = 1
            if (self.joint2 > (p4_joint_variables.joint2 - JOINT2_TOLERANCE) and self.joint2 < (p4_joint_variables.joint2 + JOINT2_TOLERANCE)):
                self.joint2_flag = 1
            if (self.joint3 > (p4_joint_variables.joint3 - JOINT3_TOLERANCE) and self.joint3 < (p4_joint_variables.joint3 + JOINT3_TOLERANCE)):
                self.joint3_flag = 1
            # Publish effort from PID controller to Robot 
            self.joint1_effort_publisher.publish(self.joint1_effort)
            self.joint2_effort_publisher.publish(self.joint2_effort)
            self.joint3_effort_publisher.publish(self.joint3_effort)

        # Stop the Robot
        self.joint1_effort = 0
        self.joint2_effort = 0
        self.joint3_effort = 0
        self.joint1_effort_publisher.publish(self.joint1_effort)
        self.joint2_effort_publisher.publish(self.joint2_effort)
        self.joint3_effort_publisher.publish(self.joint3_effort)
        self.joint1_flag = 0
        self.joint2_flag = 0
        self.joint3_flag = 0

if __name__ == '__main__':
    pid_controller = PID_Controller()
    rrp_robot = RRP_robot()