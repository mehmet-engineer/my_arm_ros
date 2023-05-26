#!/usr/bin/env python

import math

class InverseKinematics():
    
    def __init__(self):
        self.z_distance = 0.01
        self.min_joint_bounds = [0, 20, 30, 30, 0]
        self.max_joint_bounds = [180, 180, 180, 180, 90]
        self.gripper_position = 0
        self.reference = "world"

    def calculate_ik(self, x, y):
        
        j1 = 0
        j2 = 0
        j3 = 0
        j4 = 0
        j5 = 0
        
        if (abs(x) > 0.3) or (abs(x) < 0.1):
            raise ValueError("There is no ik solution! Please check X target.")
        elif (y > 0.3) or (y < 0.1):
            raise ValueError("There is no ik solution! Please check Y target.")
    
        else:

            # calculating joint 1
            if x > 0:
                j1 = 90 - math.degrees(math.atan(x/y))
            elif x == 0:
                j1 = 90
            else:
                j1 = 90 + math.degrees(math.atan(abs(x)/y))
            
            # calculating joint 2
            distance = abs(x)**2 + y**2
            
            # calculating joint 3
            
            # calculating joint 4
            j4 = 180 - 45
            
            # calculating joint 5
            j5 = 0
        
        print(j1, j2, j3, j4, j5)

robot = InverseKinematics()
robot.calculate_ik(x=-0.1, y=0.1)