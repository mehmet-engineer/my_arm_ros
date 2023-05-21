#!/usr/bin/python3

import rospy, rospkg
import actionlib
from spatialmath import SE3
import roboticstoolbox as rtb
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

"""
 @author: Mehmet Kahraman
 @date: 17.05.2023
 @about: Trajectory and inverse kinematics example script
"""

class TrajIK():
    def __init__(self):
        rospy.init_node("Trajectory_node", anonymous=True)
        rospy.loginfo("Trejectory controller Initializing...")
        
        self.init_controller()
        self.init_arm_kinematics()
    
    def init_controller(self):
        self.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
        
        self.controller = "/pos_joint_traj_controller"
        self.controller_topic = self.controller + "/follow_joint_trajectory"
        self.trajectory_msg = JointTrajectory()
        self.trajectory_msg.joint_names = self.joint_names

        self.action_client = actionlib.SimpleActionClient(self.controller_topic, FollowJointTrajectoryAction)
        self.action_client.wait_for_server()
        
        rospy.loginfo("Trajectory controller initialized.")
        rospy.sleep(1)
    
    def init_arm_kinematics(self):
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("my_arm_description")
        urdf_path = pkg_path + "/urdf/" + "robot_arm_rtb.urdf"
        self.my_arm = rtb.ERobot.URDF(urdf_path)
        
        rospy.loginfo("Robot Kinematics initialized.")
        rospy.sleep(1)
        
    def calculate_ik(self, x, y, z):
        cart_position = SE3(x, y, z)
        ik_solution = self.my_arm.ikine_LM(cart_position)
        joints_array_rad = ik_solution.q
        return joints_array_rad
        
    def go_to_joint_goal(self, angles, traj_time):
        
        rospy.loginfo("Going to trajectory goal...")
        
        start_time = rospy.Time.now()
        
        goal_point = JointTrajectoryPoint()
        goal_point.positions = angles
        goal_point.velocities = [0,0,0,0,0]
        goal_point.time_from_start = rospy.Duration.from_sec(traj_time)
        self.trajectory_msg.points.append(goal_point)
        
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = self.trajectory_msg

        self.action_client.send_goal(goal)
        success = self.action_client.wait_for_result()
        if success == True:
            rospy.loginfo("Successfully reached.")
        else:
            rospy.logwarn("There is a problem with reaching goal.")
        
        end_time = rospy.Time.now()
        elapsed_time = (start_time - end_time).to_sec()
        rospy.loginfo("elapsed_time: %f", elapsed_time)


if __name__ == '__main__':
    try:
        robot = TrajIK()
        joint_angles = robot.calculate_ik(x=0.2, y=0.2, z=0.2)
        print(joint_angles)
        robot.go_to_joint_goal(joint_angles, traj_time=6)
        
    except rospy.ROSInterruptException:
        print("ROS connection interrupted.")
        print("exiting...")