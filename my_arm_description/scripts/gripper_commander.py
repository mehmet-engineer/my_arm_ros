#!/usr/bin/python3

import rospy
from std_msgs.msg import Float64MultiArray, Bool

"""
 @author: Mehmet Kahraman
 @date: 17.05.2023
 @about: Gripper Commander
"""

class Gripper():
    def __init__(self):
        rospy.init_node("Gripper_commander_node", anonymous=True)
        rospy.loginfo("Gripper Initializing...")
        
        self.publisher = rospy.Publisher('/gripper_group_controller/command', Float64MultiArray, queue_size=10)
        rospy.Subscriber("/gripper_commander", Bool, self.callback_func)
        rospy.sleep(1)
        rospy.spin()
    
    def callback_func(self, msg):
        
        gripper_msg = Float64MultiArray()
        
        if msg.data == True:
            gripper_msg.data = [0.5, -0.5, 0.5, -0.5, -0.5, 0.5]
            self.publisher.publish(gripper_msg)
            rospy.loginfo("Gripper opened.")
        else:
            gripper_msg.data = [-0.5, 0.5, -0.5, 0.5, 0.5, -0.5]
            self.publisher.publish(gripper_msg)
            rospy.loginfo("Gripper closed.")

if __name__ == '__main__':
    try:
        robot_gripper = Gripper()
    except rospy.ROSInterruptException:
        print("\n Interrupt detected. Gripper stopped. ")