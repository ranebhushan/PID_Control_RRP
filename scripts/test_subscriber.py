#!/usr/bin/env python

import rospy
import time
from sensor_msgs.msg import JointState

class Test_Sub():

    def __init__(self):
        print("Hello")
        
        self.joint_state_subscriber = rospy.Subscriber("/rrp/joint_states", JointState, self.joint_state_callback)
        print("Class")
       
    def joint_state_callback(self, data):
        print(data)
        # print("Current Position :", self.current_value.position)
        # print("Current Velocity :", self.current_value.velocity) 

if __name__ == "__main__":
    rospy.init_node('test_sub')
    sub = Test_Sub()

    # Set rate and run
    r = rospy.Rate(60)
    while not rospy.is_shutdown():
        r.sleep()
