#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import UInt8
from sensor_msgs.msg import JointState

class JsPublisher():
    def __init__(self):
        rospy.init_node('my_joint_state_publisher_node')
        self.j0 = 0
        self.j1 = 0
        self.j2 = 0
        self.j3 = 0
        self.j4 = 0
        self.j_grip = 0
        self.init_subcribers()
        self.init_publisher()
        self.run_publisher()

    def init_subcribers(self):
        rospy.loginfo("waiting for joint state data...")
        rospy.wait_for_message('/joint_5_grip_state', UInt8, timeout=None)
        rospy.Subscriber("/joint_0_state", UInt8, self.js0_callback)
        rospy.Subscriber("/joint_1_state", UInt8, self.js1_callback)
        rospy.Subscriber("/joint_2_state", UInt8, self.js2_callback)
        rospy.Subscriber("/joint_3_state", UInt8, self.js3_callback)
        rospy.Subscriber("/joint_4_state", UInt8, self.js4_callback)
        rospy.Subscriber("/joint_5_grip_state", UInt8, self.js_grip_callback)
        rospy.loginfo("Successfully connected.")
    
    def js0_callback(self, msg):
        self.j0 = math.radians(msg.data)
        
    def js1_callback(self, msg):
        self.j1 = math.radians(msg.data-5)
    
    def js2_callback(self, msg):
        self.j2 = math.radians(msg.data+30)
    
    def js3_callback(self, msg):
        self.j3 = math.radians(msg.data)
    
    def js4_callback(self, msg):
        self.j4 = math.radians(msg.data)
    
    def js_grip_callback(self, msg):
        self.j_grip = math.radians(msg.data-5)
    
    def init_publisher(self):
        joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'gripper_joint']
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = joint_names
        self.joint_state_msg.position = [0, 0, 0, 0, 0, 0]
        self.publisher = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.rate = rospy.Rate(50)
        rospy.sleep(1)
    
    def run_publisher(self):
        while not rospy.is_shutdown():
            self.joint_state_msg.header.stamp = rospy.Time.now()
            self.joint_state_msg.position[0] = self.j0
            self.joint_state_msg.position[1] = self.j1
            self.joint_state_msg.position[2] = self.j2
            self.joint_state_msg.position[3] = self.j3
            self.joint_state_msg.position[4] = self.j4
            self.joint_state_msg.position[5] = self.j_grip
            self.publisher.publish(self.joint_state_msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        robot = JsPublisher()
    except rospy.ROSInterruptException:
        print("ROS JS Publisher closed.")