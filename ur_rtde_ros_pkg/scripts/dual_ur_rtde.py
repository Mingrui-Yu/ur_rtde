#!/usr/bin/env python

"""
- dual_ur_urde 的再次封装，其中涉及一些坐标变换和辅助函数
- 不涉及与其它 node 的通信，仅作为函数功能库
- 通用，一般无须根据任务需要进行自定义
"""

import numpy as np
import time
import copy
import math
from scipy.spatial.transform import Rotation as sciR
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf2_geometry_msgs import PointStamped, PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory
import rtde_control
import rtde_receive
import tf2_ros
import tf.transformations

from utils import *
from ur_rtde import URrtde


class DualURrtde():
    def __init__(self, arm_0, arm_1):
        self.arm_0 = arm_0
        self.arm_1 = arm_1
    

    # -----------------------------------------------------------------
    def getDualArmJointState(self):
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.header.frame_id = ""

        joint_state.name = self.arm_0.joint_names + self.arm_1.joint_names
        joint_state.position = self.arm_0.getJointAngle() + self.arm_1.getJointAngle()
        joint_state.velocity = self.arm_0.getJointVel() + self.arm_1.getJointVel()

        return joint_state


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

    
    # # ------------------------------------
    # def moveJointTrajectory(self, dual_arm_traj, asynchronous=False):
    #     # 将 dual arm trajectory 分解为 arm_0 trajectory 和 arm_1 trajectory
    #     arm_0_traj = copy.deepcopy(dual_arm_traj)
    #     arm_1_traj = copy.deepcopy(dual_arm_traj)

    #     for i in range(len(dual_arm_traj.joint_trajectory.points)):
    #         if len(dual_arm_traj.joint_trajectory.points[i].positions) != 12:
    #             rospy.logerr("Invalid dual_arm_traj command, the size is %d !", len(dual_arm_traj.joint_trajectory.points[i].positions))
            
    #         arm_0_traj.joint_trajectory.points[i].positions = dual_arm_traj.joint_trajectory.points[i].positions[:6]
    #         arm_0_traj.joint_trajectory.points[i].velocities = dual_arm_traj.joint_trajectory.points[i].velocities[:6]
    #         arm_0_traj.joint_trajectory.points[i].accelerations = dual_arm_traj.joint_trajectory.points[i].accelerations[:6]
    #         arm_0_traj.joint_trajectory.points[i].effort = dual_arm_traj.joint_trajectory.points[i].effort[:6]

    #         arm_1_traj.joint_trajectory.points[i].positions = dual_arm_traj.joint_trajectory.points[i].positions[6:]
    #         arm_1_traj.joint_trajectory.points[i].velocities = dual_arm_traj.joint_trajectory.points[i].velocities[6:]
    #         arm_1_traj.joint_trajectory.points[i].accelerations = dual_arm_traj.joint_trajectory.points[i].accelerations[6:]
    #         arm_1_traj.joint_trajectory.points[i].effort = dual_arm_traj.joint_trajectory.points[i].effort[6:]

    #     # 发送指令
    #     self.arm_0.moveJointTrajectory(arm_0_traj, asynchronous=True)
    #     self.arm_1.moveJointTrajectory(arm_1_traj, asynchronous=True)

    #     # 等到运动到达目标位置
    #     if asynchronous is False:
    #         error_thres = 1e-3
    #         arm_0_final_joint_pos = np.array(arm_0_traj.joint_trajectory.points[-1].positions)
    #         arm_1_final_joint_pos = np.array(arm_1_traj.joint_trajectory.points[-1].positions)
    #         rate = rospy.Rate(30)
    #         while not rospy.is_shutdown():
    #             arm_0_joint_pos = np.array(self.arm_0.getJointAngle())
    #             arm_1_joint_pos = np.array(self.arm_1.getJointAngle())

    #             if (np.linalg.norm(arm_0_joint_pos - arm_0_final_joint_pos) < error_thres) and \
    #                   (np.linalg.norm(arm_1_joint_pos - arm_1_final_joint_pos) < error_thres):
    #                 rospy.loginfo("Dual arm reached the final waypoints.")
    #                 break
                
    #             rate.sleep()


    # ------------------------------------
    def moveJointTrajectory(self, trajectory):
        servo_rate = 500 # Hz
        trajectory = trajectoryInterpolation(trajectory, dt=1.0/servo_rate)

        n_waypoints = len(trajectory.joint_trajectory.points)

        # Move to initial joint position with a regular moveJ
        self.arm_0.rtde_c.moveJ(trajectory.joint_trajectory.points[0].positions[:6])
        self.arm_1.rtde_c.moveJ(trajectory.joint_trajectory.points[0].positions[6:])

        rate = rospy.Rate(servo_rate)
        
        for i in range(n_waypoints):
            point = trajectory.joint_trajectory.points[i]
            self.arm_0.rtde_c.servoJ(list(point.positions[:6]), 1.0, 1.0, 1.0/servo_rate, 0.1, 300.0)
            self.arm_1.rtde_c.servoJ(list(point.positions[6:]), 1.0, 1.0, 1.0/servo_rate, 0.1, 300.0)
            rate.sleep()

        # 最后 0.5s 始终以最后一个路径点作为伺服目标
        for i in range(int(servo_rate * 0.5)):
            point = trajectory.joint_trajectory.points[-1]
            self.arm_0.rtde_c.servoJ(list(point.positions[:6]), 1.0, 1.0, 1.0/servo_rate, 0.1, 300.0)
            self.arm_1.rtde_c.servoJ(list(point.positions[6:]), 1.0, 1.0, 1.0/servo_rate, 0.1, 300.0)
            rate.sleep()

        self.arm_0.rtde_c.servoStop()
        self.arm_1.rtde_c.servoStop()


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
