#!/usr/bin/python3

import numpy as np
import math
from scipy.spatial.transform import Rotation as sciR

import rospy

import rtde_control
import rtde_receive

robot_ip = "192.168.100.50"

rtde_c = rtde_control.RTDEControlInterface(robot_ip)
rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)
# zero_configuration_q = [0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0]


actual_q = rtde_r.getActualQ()
print(actual_q)

# target_q = actual_q
# target_q[2] += 0.1

# rtde_c.moveJ(target_q, speed=0.2, acceleration=0.3)