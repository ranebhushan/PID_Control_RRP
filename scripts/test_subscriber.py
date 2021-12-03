#!/usr/bin/env python

import rospy

from sensor_msgs.msg import JointState

class Test_Sub():

    def __init__(self):
        # ROS
        # subscribe to keyboard input
        self.current_value = JointState()
        self.joint_state_subscriber = rospy.Subscriber("/rrp/joint_states", JointState, self.joint_state_callback)
       
    def joint_state_callback(self, data):

        print(self.current_value)
        # print("Current Position :", self.current_value.position)
        # print("Current Velocity :", self.current_value.velocity) 

if __name__ == "__main__":
    # Launch ros node class
    rospy.init_node('sub_test_node')
    sub = Test_Sub()

    # Set rate and run
    r = rospy.Rate(60)
    while not rospy.is_shutdown():
        r.sleep()
