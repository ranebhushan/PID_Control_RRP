#!/usr/bin/env python

from pickle import TRUE
import rospy
from time import sleep, time
from math import pi
import numpy 
from std_msgs.msg import String, Float64, Float64MultiArray, Header, _String
from sensor_msgs.msg import JointState
from rbe500_project.srv import rrpIK

# Desired end-effector positions
p1 = [0.0, 0.77, 0.34]
p2 = [-0.345, 0.425, 0.24]
p3 = [-0.67, -0.245, 0.14]
p4 = [0.77, 0.0, 0.39]

# p1 = [0.544, 0.544, 0.34]
# p2 = [0.3, 0.645, 0.24]
# p3 = [0.425, 0.344, 0.29]
# p4 = [-0.426,-0.344,0.24]

x = 0
y = 1
z = 2

# Joint1 Minimum Effort Requirement +0.2 and -0.2
JOINT1_EFFORT_LIMIT = 0.5
JOINT1_TOLERANCE = 0.001

# Joint2 Minimum Effort Requirement +0.2 and -0.2
JOINT2_EFFORT_LIMIT = 0.5
JOINT2_TOLERANCE = 0.005

# Joint3 Minimum Effort Requirement +1.1 and -3.0
JOINT3_POSITIVE_EFFORT_LIMIT = 1.1
JOINT3_NEGATIVE_EFFORT_LIMIT = -3.0
JOINT3_TOLERANCE = 0.001

class PID_Controller():

    # Intiialize variables of the controller class
    def __init__(self, P = 0.0, I = 0.0, D = 0.0, set_point=0):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.set_point = set_point
        self.error_integral = 0
        self.previous_error = 0
        
    # Calculate the updated P, I and D term
    def update(self, current_value):
        error = self.set_point - current_value
        self.error_integral = self.error_integral + error
        P_term = self.Kp*error
        I_term = self.Ki*(self.error_integral)
        D_term = self.Kd*((error - self.previous_error)/0.01)
        self.previous_error = error
        return P_term + I_term + D_term

    # Update the target point for the controller 
    def setPoint(self, set_point):
        self.set_point = set_point
        self.previous_error = 0
        self.error_integral = 0
    
    # Set the P, I and D gains for the controller
    def setPID(self, P = 1.0, I = 0.0, D = 0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D

class RRP_robot():

    # Initialize the class variables, nodes, publishers and subscribers 
    def __init__(self):

        rospy.init_node("rrp_robot_move", disable_signals=True)

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

        # Define the processing rate for ROS
        self.rate = rospy.Rate(100)

        # Subscribe to Joint States Topic
        self.current_value = JointState()
        self.data_log_counter = 0
        self.robot_path = list()
        self.joint_state_subscriber = rospy.Subscriber("/rrp/joint_states", JointState, self.joint_state_callback)

        # Define Flags to control breaking of PID Loop
        self.joint1_flag = 0
        self.joint2_flag = 0
        self.joint3_flag = 0

        # Define variables to store real time joint positions
        self.joint1 = 0
        self.joint2 = 0
        self.joint3 = 0

        # Define variables to store rela time joint velocities
        self.joint1_velocity = 0
        self.joint2_velocity = 0
        self.joint3_velocity = 0

        try:
            self.robot_move()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")

    def joint_state_callback(self, msg):
        
        # Get current joint angles [joint1, joint2, joint3] from /rrp/joint_states topic
        self.joint1 = msg.position[0]
        self.joint2 = msg.position[1]
        self.joint3 = msg.position[2]

        # Get current joint veloctiy [w1, w2, v3] from /rrp/joint_states topic
        self.joint1_velocity = msg.velocity[0]
        self.joint2_velocity = msg.velocity[1]
        self.joint3_velocity = msg.velocity[2]

    def robot_move(self):

        # Wait for the service to respond          
        rospy.wait_for_service('rrpIK', timeout=None)
        get_joint_variables = rospy.ServiceProxy('rrpIK', rrpIK)

        # Record the start time  
        tic = time()
        joint1_PID_controller = PID_Controller(P = 10.0, I = 0.00001, D = 75.0, set_point = 0)
        joint2_PID_controller = PID_Controller(P = 5.0, I = 0.00001, D = 40.0, set_point = 0)
        joint3_PID_controller = PID_Controller(P = 0.1, I = 0.0, D = 0.065, set_point = 0)

        # Define Joint Efforts for Robot to stay at origin
        self.joint1_effort = 0
        self.joint2_effort = 0
        self.joint3_effort = 0
        self.joint1_effort_publisher.publish(self.joint1_effort)
        self.joint2_effort_publisher.publish(self.joint2_effort)
        self.joint3_effort_publisher.publish(self.joint3_effort)

        # Get joint angles corresponding to position p1
        rospy.loginfo("Requesting Joint Variables for Position p1")
        rospy.loginfo("Input Position 1: X = " + str(p1[x]) + ", Y = " + str(p1[y]) + ", Z = " + str(p1[z]))
        p1_joint_variables = get_joint_variables(p1[x], p1[y], p1[z])
      
        # Define the next target position for the controller
        joint1_PID_controller.setPoint(p1_joint_variables.joint1)
        joint2_PID_controller.setPoint(p1_joint_variables.joint2)
        joint3_PID_controller.setPoint(p1_joint_variables.joint3)

        # Set P, I and D gains of the controller
        joint1_PID_controller.setPID(P = 5.0, I = 0.00001, D = 20)
        joint2_PID_controller.setPID(P = 3.0, I = 0.00001, D = 25)

        rospy.loginfo("Joint Angles (in radians) : theta1 = " + str(p1_joint_variables.joint1) + ", theta2 = " + str(p1_joint_variables.joint2) + ", d3 = " + str(p1_joint_variables.joint3))

        while (self.joint1_flag == 0 or self.joint2_flag == 0 or self.joint3_flag == 0):
            
            # Calculate control output corresponding to updated joint position 
            self.joint1_effort = joint1_PID_controller.update(self.joint1)
            self.joint2_effort = joint2_PID_controller.update(self.joint2)
            self.joint3_effort = joint3_PID_controller.update(self.joint3)

            # Define limits to controller output according to robot design and motion
            if (self.joint1_effort < 0.2 and self.joint1_effort > 0):
                self.joint1_effort = 0.2
            elif (self.joint1_effort < 0 and self.joint1_effort > -0.2):
                self.joint1_effort = -0.2

            if (self.joint2_effort < 0.2 and self.joint2_effort > 0):
                self.joint2_effort = 0.2
            elif (self.joint2_effort < 0 and self.joint2_effort > -0.2):
                self.joint2_effort = -0.2
            
            if (self.joint3_effort > 0):
                self.joint3_effort = JOINT3_POSITIVE_EFFORT_LIMIT
            elif (self.joint3_effort < 0):
                self.joint3_effort = JOINT3_NEGATIVE_EFFORT_LIMIT

            # print("Joint1: ",self.joint1_effort, " Joint2: ", self.joint2_effort, " Joint3: ", self.joint3_effort)

            # Breaking condition for PID loop to exit
            if ((self.joint1 <= (p1_joint_variables.joint1 - JOINT1_TOLERANCE)) and (self.joint1 >= (p1_joint_variables.joint1 + JOINT1_TOLERANCE))):
                self.joint1_flag = 0
            if ((self.joint1 > (p1_joint_variables.joint1 - JOINT1_TOLERANCE)) and (self.joint1 < (p1_joint_variables.joint1 + JOINT1_TOLERANCE))):
                self.joint1_flag = 1
            
            if ((self.joint2 <= (p1_joint_variables.joint2 - JOINT2_TOLERANCE)) and (self.joint2 >= (p1_joint_variables.joint2 + JOINT2_TOLERANCE))):
                self.joint2_flag = 0
            if ((self.joint2 > (p1_joint_variables.joint2 - JOINT2_TOLERANCE)) and (self.joint2 < (p1_joint_variables.joint2 + JOINT2_TOLERANCE))):
                self.joint2_flag = 1

            if (self.joint3 > (p1_joint_variables.joint3 - JOINT3_TOLERANCE) and self.joint3 < (p1_joint_variables.joint3 + JOINT3_TOLERANCE)):
                self.joint3_flag = 1

            # Publish controller output effort to the joint effort controller topic
            self.joint1_effort_publisher.publish(self.joint1_effort)
            self.joint2_effort_publisher.publish(self.joint2_effort)
            self.joint3_effort_publisher.publish(self.joint3_effort)
        
        # Set the effort to zero to stop the robot
        self.joint1_effort = 0.0
        self.joint2_effort = 0.0
        self.joint3_effort = 0.0

        # Publish the zero effort to joint effort controller topic to stop the robot motion further
        self.joint1_effort_publisher.publish(self.joint1_effort)
        self.joint2_effort_publisher.publish(self.joint2_effort)
        self.joint3_effort_publisher.publish(self.joint3_effort)

        # Reset the loop flags for further processing
        self.joint1_flag = 0
        self.joint2_flag = 0
        self.joint3_flag = 0

        # Acknowledge robot's movement to target position
        rospy.loginfo("HEY I REACHED POSITION 1")
        rospy.loginfo("Current Joint1 = " + str(self.joint1) + ", Joint2 = " + str(self.joint2) + ", Joint3 = " + str(self.joint3))
        
        # Stop for 1 second at each desired configuration
        sleep(1)

        # Get joint angles corresponding to position p2
        rospy.loginfo("Requesting Joint Variables for Position p2")
        rospy.loginfo("Input Position 2: X = " + str(p2[x]) + ", Y = " + str(p2[y]) + ", Z = " + str(p2[z]))
        p2_joint_variables = get_joint_variables(p2[x], p2[y], p2[z])
       
        # Define the next target position for the controller
        joint1_PID_controller.setPoint(p2_joint_variables.joint1)
        joint2_PID_controller.setPoint(p2_joint_variables.joint2)
        joint3_PID_controller.setPoint(p2_joint_variables.joint3)
        rospy.loginfo("Joint Angles (in radians) : theta1 = " + str(p2_joint_variables.joint1) + ", theta2 = " + str(p2_joint_variables.joint2) + ", d3 = " + str(p2_joint_variables.joint3))

        # Set P, I and D gains of the controller
        joint1_PID_controller.setPID(P = 3.0, I = 0.00001, D = 30.0)
        joint2_PID_controller.setPID(P = 2.0, I = 0.00001, D = 10.0)

        while (self.joint1_flag == 0 or self.joint2_flag == 0 or self.joint3_flag == 0):
            
            # Calculate control output corresponding to updated joint position 
            self.joint1_effort = joint1_PID_controller.update(self.joint1)
            self.joint2_effort = joint2_PID_controller.update(self.joint2)
            self.joint3_effort = joint3_PID_controller.update(self.joint3)

            # Define limits to controller output according to robot design and motion
            if (self.joint1_effort < 0.2 and self.joint1_effort > 0):
                self.joint1_effort = 0.2
            elif (self.joint1_effort < 0 and self.joint1_effort > -0.2):
                self.joint1_effort = -0.2

            if (self.joint2_effort < 0.2 and self.joint2_effort > 0):
                self.joint2_effort = 0.2
            elif (self.joint2_effort < 0 and self.joint2_effort > -0.2):
                self.joint2_effort = -0.2
            
            if (self.joint3_effort > 0):
                self.joint3_effort = JOINT3_POSITIVE_EFFORT_LIMIT
            elif (self.joint3_effort < 0):
                self.joint3_effort = JOINT3_NEGATIVE_EFFORT_LIMIT

            # print("Joint1: ",self.joint1_effort, " Joint2: ", self.joint2_effort, " Joint3: ", self.joint3_effort)

            # Breaking condition for PID loop to exit
            if ((self.joint1 <= (p2_joint_variables.joint1 - JOINT1_TOLERANCE)) and (self.joint1 >= (p2_joint_variables.joint1 + JOINT1_TOLERANCE))):
                self.joint1_flag = 0
            if ((self.joint1 > (p2_joint_variables.joint1 - JOINT1_TOLERANCE)) and (self.joint1 < (p2_joint_variables.joint1 + JOINT1_TOLERANCE))):
                self.joint1_flag = 1
            
            if ((self.joint2 <= (p2_joint_variables.joint2 - JOINT2_TOLERANCE)) and (self.joint2 >= (p2_joint_variables.joint2 + JOINT2_TOLERANCE))):
                self.joint2_flag = 0
            if ((self.joint2 > (p2_joint_variables.joint2 - JOINT2_TOLERANCE)) and (self.joint2 < (p2_joint_variables.joint2 + JOINT2_TOLERANCE))):
                self.joint2_flag = 1

            if (self.joint3 > (p2_joint_variables.joint3 - JOINT3_TOLERANCE) and self.joint3 < (p2_joint_variables.joint3 + JOINT3_TOLERANCE)):
                self.joint3_flag = 1

            # Publish controller output effort to the joint effort controller topic
            self.joint1_effort_publisher.publish(self.joint1_effort)
            self.joint2_effort_publisher.publish(self.joint2_effort)
            self.joint3_effort_publisher.publish(self.joint3_effort)

        # Acknowledge robot reaching to the target position once the loop breaks
        rospy.loginfo("HEY I REACHED POSITION 2")
        rospy.loginfo("Current Joint1 = " + str(self.joint1) + ", Joint2 = " + str(self.joint2) + ", Joint3 = " + str(self.joint3))
        
        # Set the effort to zero to stop the robot
        self.joint1_effort = 0.0
        self.joint2_effort = 0.0
        self.joint3_effort = 0.0

        # Publish the zero effort to joint effort controller topic to stop the robot motion further
        self.joint1_effort_publisher.publish(self.joint1_effort)
        self.joint2_effort_publisher.publish(self.joint2_effort)
        self.joint3_effort_publisher.publish(self.joint3_effort)

        # Reset the loop flags for further processing
        self.joint1_flag = 0
        self.joint2_flag = 0
        self.joint3_flag = 0

        # Stop for 1 second at each desired configuration
        sleep(1)

        # Get joint angles corresponding to position p3
        rospy.loginfo("Requesting Joint Variables for Position p3")
        rospy.loginfo("Input Position 3: X = " + str(p3[x]) + ", Y = " + str(p3[y]) + ", Z = " + str(p3[z]))
        p3_joint_variables = get_joint_variables(p3[x], p3[y], p3[z])
        
        # Define the next target position for the controller
        joint1_PID_controller.setPoint(p3_joint_variables.joint1)
        joint2_PID_controller.setPoint(p3_joint_variables.joint2)
        joint3_PID_controller.setPoint(p3_joint_variables.joint3)
        rospy.loginfo("Joint Angles (in radians) : theta1 = " + str(p3_joint_variables.joint1) + ", theta2 = " + str(p3_joint_variables.joint2) + ", d3 = " + str(p3_joint_variables.joint3))
        
        # Set P, I and D gains of the controller
        joint1_PID_controller.setPID(P = 5.0, I = 0.00001, D = 90)
        joint2_PID_controller.setPID(P = 5.0, I = 0.00001, D = 20)

        while (self.joint1_flag == 0 or self.joint2_flag == 0 or self.joint3_flag == 0):
            
            # Calculate control output corresponding to updated joint position 
            self.joint1_effort = joint1_PID_controller.update(self.joint1)
            self.joint2_effort = joint2_PID_controller.update(self.joint2)
            self.joint3_effort = joint3_PID_controller.update(self.joint3)

            # Define limits to controller output according to robot design and motion
            if (self.joint1_effort > 2.0):
                self.joint1_effort = 10.0
            elif (self.joint1_effort < -5.0):
                self.joint1_effort = -5.0
            if (self.joint1_effort < 0.2 and self.joint1_effort > 0):
                self.joint1_effort = 0.2
            elif (self.joint1_effort < 0 and self.joint1_effort > -0.2):
                self.joint1_effort = -0.2

            if (self.joint2_effort < JOINT2_EFFORT_LIMIT and self.joint2_effort > 0):
                self.joint2_effort = 0.2
            elif (self.joint2_effort < 0 and self.joint2_effort > -JOINT2_EFFORT_LIMIT):
                self.joint2_effort = -0.2

            if (self.joint3_effort > 0):
                self.joint3_effort = JOINT3_POSITIVE_EFFORT_LIMIT
            elif (self.joint3_effort < 0):
                self.joint3_effort = JOINT3_NEGATIVE_EFFORT_LIMIT

            # print("Joint1: ",self.joint1_effort, " Joint2: ", self.joint2_effort, " Joint3: ", self.joint3_effort)

            # Breaking condition for PID loop to exit
            if ((self.joint1 <= (p3_joint_variables.joint1 - JOINT1_TOLERANCE)) and (self.joint1 >= (p3_joint_variables.joint1 + JOINT1_TOLERANCE))):
                self.joint1_flag = 0
            if ((self.joint1 > (p3_joint_variables.joint1 - JOINT1_TOLERANCE)) and (self.joint1 < (p3_joint_variables.joint1 + JOINT1_TOLERANCE))):
                self.joint1_flag = 1
            
            if ((self.joint2 <= (p3_joint_variables.joint2 - JOINT2_TOLERANCE)) and (self.joint2 >= (p3_joint_variables.joint2 + JOINT2_TOLERANCE))):
                self.joint2_flag = 0
            if ((self.joint2 > (p3_joint_variables.joint2 - JOINT2_TOLERANCE)) and (self.joint2 < (p3_joint_variables.joint2 + JOINT2_TOLERANCE))):
                self.joint2_flag = 1

            if (self.joint3 > (p3_joint_variables.joint3 - JOINT3_TOLERANCE) and self.joint3 < (p3_joint_variables.joint3 + JOINT3_TOLERANCE)):
                self.joint3_flag = 1

            # Publish controller output effort to the joint effort controller topic
            self.joint1_effort_publisher.publish(self.joint1_effort)
            self.joint2_effort_publisher.publish(self.joint2_effort)
            self.joint3_effort_publisher.publish(self.joint3_effort)

        # Set the effort to zero to stop the robot
        self.joint1_effort = 0.0
        self.joint2_effort = 0.0
        self.joint3_effort = 0.0

        # Publish the zero effort to joint effort controller topic to stop the robot motion further
        self.joint1_effort_publisher.publish(self.joint1_effort)
        self.joint2_effort_publisher.publish(self.joint2_effort)
        self.joint3_effort_publisher.publish(self.joint3_effort)

        # Reset the loop flags for further processing
        self.joint1_flag = 0
        self.joint2_flag = 0
        self.joint3_flag = 0

        # Acknowledge robot reaching to the target position once the loop breaks
        rospy.loginfo("HEY I REACHED POSITION 3")
        rospy.loginfo("Current Joint1 = " + str(self.joint1) + ", Joint2 = " + str(self.joint2) + ", Joint3 = " + str(self.joint3))

        # Stop for 1 second at each desired configuration
        sleep(1)

        # Get joint angles corresponding to position p4
        rospy.loginfo("Requesting Joint Variables for Position p4")
        rospy.loginfo("Input Position 4: X = " + str(p4[x]) + ", Y = " + str(p4[y]) + ", Z = " + str(p4[z]))
        p4_joint_variables = get_joint_variables(p4[x], p4[y], p4[z])
        
        # Define the next target position for the controller
        joint1_PID_controller.setPoint(p4_joint_variables.joint1)
        joint2_PID_controller.setPoint(p4_joint_variables.joint2)
        joint3_PID_controller.setPoint(p4_joint_variables.joint3)
        rospy.loginfo("Joint Angles (in radians) : theta1 = " + str(p4_joint_variables.joint1) + ", theta2 = " + str(p4_joint_variables.joint2) + ", d3 = " + str(p4_joint_variables.joint3))
        
        # Set P, I and D gains of the controller
        joint1_PID_controller.setPID(P = 1.0, I = 0.00001, D = 50)
        joint2_PID_controller.setPID(P = 5.0, I = 0.00001, D = 20)

        while (self.joint1_flag == 0 or self.joint2_flag == 0 or self.joint3_flag == 0):

            # Calculate control output corresponding to updated joint position 
            self.joint1_effort = joint1_PID_controller.update(self.joint1)
            self.joint2_effort = joint2_PID_controller.update(self.joint2)
            self.joint3_effort = joint3_PID_controller.update(self.joint3)

            # Define limits to controller output according to robot design and motion
            if (self.joint1_effort > 5.0):
                self.joint1_effort = 5.0
            elif (self.joint1_effort < 0):
                self.joint1_effort = -10.0
            if (self.joint1_effort < 0.2 and self.joint1_effort > 0):
                self.joint1_effort = 0.2
            elif (self.joint1_effort < 0 and self.joint1_effort > -0.2):
                self.joint1_effort = -0.2

            if (self.joint2_effort < 0.2 and self.joint2_effort > 0):
                self.joint2_effort = 0.2
            elif (self.joint2_effort < 0 and self.joint2_effort > -0.2):
                self.joint2_effort = -0.2
            
            if (self.joint3_effort > 0):
                self.joint3_effort = JOINT3_POSITIVE_EFFORT_LIMIT
            elif (self.joint3_effort < 0):
                self.joint3_effort = JOINT3_NEGATIVE_EFFORT_LIMIT

            # print("Joint1: ",self.joint1_effort, " Joint2: ", self.joint2_effort, " Joint3: ", self.joint3_effort)

            # Breaking condition for PID loop to exit
            if ((self.joint1 <= (p4_joint_variables.joint1 - JOINT1_TOLERANCE)) and (self.joint1 >= (p4_joint_variables.joint1 + JOINT1_TOLERANCE))):
                self.joint1_flag = 0
            if ((self.joint1 > (p4_joint_variables.joint1 - JOINT1_TOLERANCE)) and (self.joint1 < (p4_joint_variables.joint1 + JOINT1_TOLERANCE))):
                self.joint1_flag = 1
            
            if (self.joint2 <= (p4_joint_variables.joint2 - JOINT2_TOLERANCE) and self.joint2 >= (p4_joint_variables.joint2 + JOINT2_TOLERANCE)):
                self.joint2_flag = 0
            if (self.joint2 > (p4_joint_variables.joint2 - JOINT2_TOLERANCE) and self.joint2 < (p4_joint_variables.joint2 + JOINT2_TOLERANCE)):
                self.joint2_flag = 1

            if (self.joint3 > (p4_joint_variables.joint3 - JOINT3_TOLERANCE) and self.joint3 < (p4_joint_variables.joint3 + JOINT3_TOLERANCE)):
                self.joint3_flag = 1

            # Publish controller output effort to the joint effort controller topic
            self.joint1_effort_publisher.publish(self.joint1_effort)
            self.joint2_effort_publisher.publish(self.joint2_effort)
            self.joint3_effort_publisher.publish(self.joint3_effort)

        # Set the effort to zero to stop the robot
        self.joint1_effort = 0.0
        self.joint2_effort = 0.0
        self.joint3_effort = 0.0

        # Publish the zero effort to joint effort controller topic to stop the robot motion further
        self.joint1_effort_publisher.publish(self.joint1_effort)
        self.joint2_effort_publisher.publish(self.joint2_effort)
        self.joint3_effort_publisher.publish(self.joint3_effort)

        # Reset the loop flags for further processing
        self.joint1_flag = 0
        self.joint2_flag = 0
        self.joint3_flag = 0
        
        # Acknowledge robot reaching to the target position once the loop breaks
        rospy.loginfo("HEY I REACHED POSITION 4")
        rospy.loginfo("Current Joint1 = " + str(self.joint1) + ", Joint2 = " + str(self.joint2) + ", Joint3 = " + str(self.joint3))
        
        # Record the current time in order to calculate the total task completion time
        toc = time()
        rospy.loginfo("TASK COMPLETE")
        rospy.loginfo("Elapsed time is " + str(toc-tic) + " seconds")

if __name__ == '__main__':
    pid_controller = PID_Controller()
    rrp_robot = RRP_robot()