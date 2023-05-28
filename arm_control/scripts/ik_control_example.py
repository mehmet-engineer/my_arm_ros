#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import UInt16MultiArray, UInt8

class InverseKinematics():
    
    def __init__(self):
        self.min_joint_bounds = [0, 20, 30, 30, 0]
        self.max_joint_bounds = [180, 180, 180, 180, 90]
        self.reference = "world"
        self.eef = "tool_link"
        self.link_1 = 0.1
        self.link_2 = 0.091
        self.eef_link_long = 0.145
        self.eef_link_short = 0.024
        self.z_horizontal = 0.08
        self.target_z = 0.02
        self.z_distance = self.z_horizontal - self.target_z

    def calculate_ik(self, x, y):
        j1 = 0
        j2 = 0
        j3 = 0
        j4 = 0
        j5 = 0
        if (abs(x) > 0.3):
            raise ValueError("There is no ik solution! Robot can access 0.12m - 0.3m radius. Please check X target.")
        elif (y > 0.3):
            raise ValueError("There is no ik solution! Robot can access 0.12m - 0.3m radius. Please check Y target.")
        elif math.sqrt(abs(x)**2 + y**2) > 0.3:
            raise ValueError("There is no ik solution! Robot can access max 0.3 m radius.")
        elif math.sqrt(abs(x)**2 + y**2) < 0.12:
            raise ValueError("There is no ik solution! Robot can access min 0.12 m radius.")
        else:
            # calculating joint 1
            if x > 0:
                j1 = 90 - int(math.degrees(math.atan(x/y)))
            elif x == 0:
                j1 = 90
            else:
                j1 = 90 + int(math.degrees(math.atan(abs(x)/y)))

            # calculating joint 2 and 3
            distance = math.sqrt(abs(x)**2 + y**2)
            b1 = self.eef_link_long * math.cos(math.radians(45))
            b2 = self.eef_link_short * math.cos(math.radians(45))
            b = b1 + b2
            a_dist = distance - b
            diff_thresh = 0.001

            key = True
            for alpha2 in range(25, 90, 1):
                d1 = self.link_1 * math.cos(math.radians(alpha2))
                d2 = a_dist - d1
                for alpha3 in range(55, 10, -1):
                    mid_line = self.link_1 * math.sin(math.radians(alpha2)) - 0.025
                    calculated_mid = self.link_2 * math.sin(math.radians(alpha3))
                    if calculated_mid > mid_line:
                        continue
                    calculated_d2 = self.link_2 * math.cos(math.radians(alpha3))
                    if (calculated_d2 > d2-diff_thresh) and (calculated_d2 < d2+diff_thresh):
                        key = False
                        break
                if key == False:
                    break

            j2 = 20 + alpha2
            j3 = 140 - (alpha2 + alpha3)
            
            # calculating joint 4
            j4 = 180 - 45
            
            # calculating joint 5
            j5 = 0
        
        print("radius distance:", round(distance,3))
        return [j1, j2, j3, j4, j5]

def main():
    rospy.init_node("ik_example_node")
    pos_publisher = rospy.Publisher("/pos_command_topic", UInt16MultiArray, queue_size=1)
    gripper_publisher = rospy.Publisher("/gripper_command_topic", UInt8, queue_size=1)

    robot = InverseKinematics()
    """ your codes to calculate x and y """
    x = 0.28
    y = 0.02
    angles = robot.calculate_ik(x, y)
    
    pos_msg = UInt16MultiArray()
    pos_msg.data = [0, 0, 0, 0, 0]
    pos_msg.data[0] = angles[0]
    pos_msg.data[1] = angles[1]
    pos_msg.data[2] = angles[2]
    pos_msg.data[3] = angles[3]
    pos_msg.data[4] = angles[4]

    open_grip_msg = UInt8()
    open_grip_msg.data = 1
    close_grip_msg = UInt8()
    close_grip_msg.data = 2

    rospy.loginfo("going kinematic target X:%f and Y:%f ...", x, y)
    rospy.loginfo("calculated angle target J0:%d J1:%d J3:%d J4:%d J5:%d", angles[0], angles[1], angles[2], angles[3], angles[4])
    pos_publisher.publish(pos_msg)
    gripper_publisher.publish(close_grip_msg)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("ROS ik example closed.")