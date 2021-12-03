#!/usr/bin/env python

import rospy
from time import sleep
from math import pi
import numpy 
from std_msgs.msg import String, Float64, Float64MultiArray, Header, _String
from sensor_msgs.msg import JointState


class PID_Controller():

    def __init__(self, P = 0.0, I = 0.0, D = 0.0, set_point=0):
        self.Kp = P
        self.Ki - I
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

        # Publish Joint Velocity to Joint1 Effor Controller
        self.joint1_velocity = Float64()
        self.joint1_velocity.data = 0
        self.joint1_velocity_publisher = rospy.Publisher("/rrp/joint1_effort_controller/command", Float64, queue_size=10)

        # Publish Joint Velocity to Joint2 Effor Controller
        self.joint2_velocity = Float64()
        self.joint2_velocity.data = 0
        self.joint2_velocity_publisher = rospy.Publisher("/rrp/joint2_effort_controller/command", Float64, queue_size=10)

        # Publish Joint Velocity to Joint3 Effor Controller
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
            self.run()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")
        finally:
            # save trajectory into csv file
            numpy.savetxt('trajectory.csv', numpy.array(self.trajectory), fmt='%f', delimiter=',')

    def joint_state_callback(self, msg):
        # Get current end-effector position [x, y, z] from /rrp/joint_states topic
        # Get current joint veloctiy [joint1, joint2, joint3] from /rrp/joint_states topic

        # Below 2 lines are doubtful
        # position = [msg.position[0], msg.position[1], msg.position[2]]
        # joint_velocity = [msg.velocity[0], msg.velocity[1], msg.velocity[2]]

        self.x_position = msg.current_value.position[0]
        self.y_position = msg.current_value.position[1]
        self.z_position = msg.current_value.position[2]

        self.joint1_velocity = msg.current_value.velocity[0]
        self.joint2_velocity = msg.current_value.velocity[1]
        self.joint3_velocity = msg.current_value.velocity[2]

        self.data_log_counter += 1
        if self.data_log_counter == 100:
            self.data_log_counter = 0
            self.robot_path.append([self.pose.x, self.pose.y]) # save trajectory
            rospy.loginfo("X = " + str(self.x_position) +"; Y = " + str(self.y_position) + "; Z = " + str(self.z_position))        

    def run(self):
        # self.vel.linear.x = 1.0
        # self.vel.angular.z = 0.0
        # self.vel_pub.publish(self.vel)
        # remove the code above and add your code here to adjust your movement based on 2D pose feedback
        joint1_PID_controller = PID_Controller(P = 1.0, I = 1.0, D = 1.0, set_point = 0)
        joint2_PID_controller = PID_Controller(P = 1.0, I = 1.0, D = 1.0, set_point = 0)
        joint3_PID_controller = PID_Controller(P = 1.0, I = 1.0, D = 1.0, set_point = 0)   

if __name__ == '__main__':
    pid_controller = PID_Controller()
    rrp_robot = RRP_robot()