#!/usr/bin/env python

"""
- dual_ur_urde 的再次封装，其中涉及一些坐标变换和辅助函数
- 不涉及与其它 node 的通信，仅作为函数功能库
- 通用，一般无须根据任务需要进行自定义
"""

import numpy as np
import time
import math
from scipy.spatial.transform import Rotation as sciR
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
import rtde_control
import rtde_receive
import tf2_ros
from tf2_geometry_msgs import PointStamped, PoseStamped
from geometry_msgs.msg import TwistStamped
from moveit_msgs.msg import RobotTrajectory
import tf.transformations

from utils import *
from ur_rtde import URrtde


class DualURrtde():
    def __init__(self, arm_0, arm_1):
        self.arm_0 = arm_0
        self.arm_1 = arm_1

    # ------------------------------------
    def moveToTcpPose(self, arm_0_target_posestamped, arm_1_target_posestamped, speed=0.1, acceleration=0.5, asynchronous=False):
        self.arm_0.moveToTcpPose(arm_0_target_posestamped, speed, acceleration, asynchronous=True)
        self.arm_1.moveToTcpPose(arm_1_target_posestamped, speed, acceleration, asynchronous=True)

        # 等到运动到达目标位置
        if asynchronous is False:
            error_thres = 1e-3
            rate = rospy.Rate(30)
            while not rospy.is_shutdown():
                arm_0_posestamped = self.arm_0.getTcpPose(arm_0_target_posestamped.header.frame_id)
                arm_1_posestamped = self.arm_1.getTcpPose(arm_1_target_posestamped.header.frame_id)

                if (distanceBetweenRosPoses(arm_0_posestamped.pose, arm_0_target_posestamped.pose) < error_thres) and \
                     (distanceBetweenRosPoses(arm_1_posestamped.pose, arm_1_target_posestamped.pose) < error_thres):
                    rospy.loginfo("Dual arm reached the target poses.")
                    break

                rate.sleep()

    
    # ------------------------------------
    def moveJointTrajectory(self, dual_arm_traj, asynchronous=False):
        # 将 dual arm trajectory 分解为 arm_0 trajectory 和 arm_1 trajectory
        arm_0_traj = dual_arm_traj
        arm_1_traj = dual_arm_traj
        for i in range(len(dual_arm_traj.joint_trajectory.points)):
            if len(dual_arm_traj.joint_trajectory.points[i].positions) != 12:
                rospy.logerr("Invalid dual_arm_traj command, the size is %d !", len(dual_arm_traj.joint_trajectory.points[i].positions))
            
            arm_0_traj.joint_trajectory.points[i].positions = dual_arm_traj.joint_trajectory.points[i].positions[:6]
            arm_0_traj.joint_trajectory.points[i].velocities = dual_arm_traj.joint_trajectory.points[i].velocities[:6]
            arm_0_traj.joint_trajectory.points[i].accelerations = dual_arm_traj.joint_trajectory.points[i].accelerations[:6]
            arm_0_traj.joint_trajectory.points[i].effort = dual_arm_traj.joint_trajectory.points[i].effort[:6]

            arm_1_traj.joint_trajectory.points[i].positions = dual_arm_traj.joint_trajectory.points[i].positions[6:]
            arm_1_traj.joint_trajectory.points[i].velocities = dual_arm_traj.joint_trajectory.points[i].velocities[6:]
            arm_1_traj.joint_trajectory.points[i].accelerations = dual_arm_traj.joint_trajectory.points[i].accelerations[6:]
            arm_1_traj.joint_trajectory.points[i].effort = dual_arm_traj.joint_trajectory.points[i].effort[6:]

        # 发送指令
        self.arm_0.moveJointTrajectory(arm_0_traj, asynchronous=True)
        self.arm_1.moveJointTrajectory(arm_1_traj, asynchronous=True)

        # 等到运动到达目标位置
        if asynchronous is False:
            error_thres = 1e-3
            arm_0_final_joint_pos = np.array(arm_0_traj.joint_trajectory.points[-1].positions)
            arm_1_final_joint_pos = np.array(arm_1_traj.joint_trajectory.points[-1].positions)
            rate = rospy.Rate(30)
            while not rospy.is_shutdown():
                arm_0_joint_pos = np.array(self.arm_0.getJointAngle())
                arm_1_joint_pos = np.array(self.arm_1.getJointAngle())

                if (np.linalg.norm(arm_0_joint_pos - arm_0_final_joint_pos) < error_thres) and \
                      (np.linalg.norm(arm_1_joint_pos - arm_1_final_joint_pos) < error_thres):
                    rospy.loginfo("Dual arm reached the final waypoints.")
                    break
                
                rate.sleep()


    # ------------------------------------
    def robotShutDown(self):
        self.arm_0.robotShutDown()
        self.arm_1.robotShutDown()


# --------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    try:
        rospy.init_node("dual_ur_node")
        arm_0 = URrtde("192.168.100.50", prefix="arm_0_", base_frame_id="arm_0_base", world_frame_id="world")
        arm_1 = URrtde("192.168.101.50", prefix="arm_1_", base_frame_id="arm_1_base", world_frame_id="world")

        dual_arm = DualURrtde(arm_0, arm_1)

        arm_0_tcp_posestamped = dual_arm.arm_0.getTcpPose("world")
        arm_1_tcp_posestamped = dual_arm.arm_1.getTcpPose("world")

        arm_0_target_tcp_posestamped = arm_0_tcp_posestamped
        arm_1_target_tcp_posestamped = arm_1_tcp_posestamped
        arm_0_target_tcp_posestamped.pose.position.z += 0.05
        arm_1_target_tcp_posestamped.pose.position.z += 0.05

        dual_arm.moveToTcpPose(arm_0_target_tcp_posestamped, arm_1_target_tcp_posestamped)


        dual_arm.robotShutDown()

        




    except rospy.ROSInterruptException:
        pass
