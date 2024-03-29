#!/usr/bin/env python

import math

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
        if (abs(x) > 0.3) or (abs(x) < 0.12):
            raise ValueError("There is no ik solution! Robot can access 0.12m - 0.3m radius. Please check X target.")
        elif (y > 0.3) or (y < 0.12):
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

#robot = InverseKinematics()
#print(robot.calculate_ik(-0.25, 0.15))