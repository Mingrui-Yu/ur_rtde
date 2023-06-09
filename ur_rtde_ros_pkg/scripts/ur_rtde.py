#!/usr/bin/env python

"""
- ur_urde 的再次封装，其中涉及一些坐标变换和辅助函数
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
import tf.transformations
from utils import *


# ----------------------------------------------------------------------
class URrtde(object):
    def __init__(self, ip, prefix="", home_pose=None, tcp_offset=None, base_frame_id="", world_frame_id=""):
        self.prefix = prefix
        self.home_pose = home_pose
        self.rtde_c = rtde_control.RTDEControlInterface(ip)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(ip)
        self.base_frame_id = base_frame_id
        self.world_frame_id = world_frame_id

        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)
        # self.rot_mat_base2world = self.getBase2WorldRotationMatrix()

        # 设定工具末端相对于法兰盘中心的pose （手动测出来的）
        if tcp_offset is not None:
            self.rtde_c.setTcp(tcp_offset)

        # joint names
        self.joint_names = ["shoulder_pan_joint", 
                            "shoulder_lift_joint",
                            "elbow_joint",
                            "wrist_1_joint",
                            "wrist_2_joint",
                            "wrist_3_joint"]
        for i in range(len(self.joint_names)):
            self.joint_names[i] = self.prefix + self.joint_names[i]

    
    # ----------------------------------------------------------------------
    def getJointAngle(self):
        return self.rtde_r.getActualQ()


    # ----------------------------------------------------------------------
    def getJointVel(self):
        return self.rtde_r.getActualQd()
    
    
    # ----------------------------------------------------------------------
    def getFrame1ToFrame2RotationMatrix(self, frame_1_id, frame_2_id):
        while not rospy.is_shutdown() and not self.tfBuffer.can_transform(frame_2_id, frame_1_id, rospy.Time.now()):
            rospy.loginfo_once("Waiting for TF tranform ...")

        trans = self.tfBuffer.lookup_transform(frame_2_id, frame_1_id, rospy.Time.now())
        q = np.array([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
        trans_T = tf.transformations.quaternion_matrix(q)
        rotation_matrix = np.array(trans_T[0:3, 0:3])

        return rotation_matrix
    
    
    # ----------------------------------------------------------------------
    # 获取base2world的旋转矩阵，用于速度变换
    def getBase2WorldRotationMatrix(self):
        if (not self.world_frame_id) or (not self.base_frame_id): 
            rospy.logwarn("Base frame id or world frame id has not been set. Cannot calculate the base2world rotation matrix.")
        return self.getFrame1ToFrame2RotationMatrix(self.base_frame_id, self.world_frame_id)
    
    
    # ----------------------------------------------------------------------
    def getTcpPoseInBase(self):
        """
        output: 
            tcp_pose_ros: PoseStamped msg
        """
        tcp_pose = np.array(self.rtde_r.getActualTCPPose())
        tcp_pose_ros = PoseStamped()
        tcp_pose_ros.header.stamp = rospy.Time.now()
        tcp_pose_ros.header.frame_id = self.base_frame_id

        tcp_pose_ros.pose.position.x = tcp_pose[0]
        tcp_pose_ros.pose.position.y= tcp_pose[1]
        tcp_pose_ros.pose.position.z = tcp_pose[2]

        tcp_ori = np.array(tcp_pose[3:])
        r = sciR.from_rotvec(tcp_ori)
        q = r.as_quat()
        tcp_pose_ros.pose.orientation.x = q[0]
        tcp_pose_ros.pose.orientation.y = q[1]
        tcp_pose_ros.pose.orientation.z= q[2]
        tcp_pose_ros.pose.orientation.w = q[3]

        return tcp_pose_ros


    # ----------------------------------------------------------------------
    def getTcpPose(self, target_frame_id):
        posestamped_in_base = self.getTcpPoseInBase()

        # 将base坐标系下的pose转为target_frame_id坐标系下
        try:
            posestamped = self.tfBuffer.transform(posestamped_in_base, target_frame_id)
        except:
            rospy.logerr("Cannot transform from " + posestamped_in_base.header.frame_id \
                + " to " + target_frame_id + ".")

        return posestamped


    # ----------------------------------------------------------------------
    def robotShutDown(self):
        self.rtde_c.speedStop(1)
        self.rtde_c.stopScript()
        rospy.loginfo(self.prefix + " is shutted down.")
        
    
    # ----------------------------------------------------------------------
    def attainHome(self, b_async=False):
        if self.home_pose is not None:
            # self.rtde_c.moveJ(self.home_pose, speed=0.2, async=b_async)
            self.rtde_c.moveL_FK(self.home_pose, speed=0.1, acceleration=0.5, asynchronous=b_async) # Move to position (linear in tool-space)
            print(self.prefix + " move to the home pose.")
            
            
    # ----------------------------------------------------------------------
    def controlJointVel(self, joint_vel, acceleration=2.0):
        if(len(joint_vel) != 6):
            rospy.logerr("controlJointVel(): the dimension of joint_vel is wrong.")
            
        self.rtde_c.speedJ(joint_vel, acceleration=acceleration, time=0.001) # time=0 会报错
        
        
    # ----------------------------------------------------------------------
    def controlTcpVel(self, tcp_twist_stamped, acceleration=1.0):
        target_frame_id = tcp_twist_stamped.header.frame_id
        twist = tcp_twist_stamped.twist
        
        v_w = np.array([twist.linear.x, twist.linear.y, twist.linear.z]).reshape(-1, 1)
        omega_w = np.array([twist.angular.x, twist.angular.y, twist.angular.z]).reshape(-1, 1)
        
        # calculate the rotation matrix between the base frame and the target frame
        rot_mat_base2target = self.getFrame1ToFrame2RotationMatrix(self.base_frame_id, target_frame_id)
        
        # transform the twist in the target frame to the base frame
        v_b = np.dot(rot_mat_base2target.T, v_w)
        omega_b = np.dot(rot_mat_base2target.T, omega_w)
        
        tcp_velocity_base = [v_b[0, 0], v_b[1, 0], v_b[2, 0], omega_b[0, 0], omega_b[1, 0], omega_b[2, 0]]
        self.rtde_c.speedL(tcp_velocity_base, acceleration=acceleration)


    # ----------------------------------------------------------------------
    def moveToTcpPose(self, target_posestamped, speed=0.1, acceleration=0.5, asynchronous=False):
        # 将pose转到base坐标系下
        try:
            target_posestamped_in_base = self.tfBuffer.transform(target_posestamped, self.base_frame_id)
        except:
            rospy.logerr("Cannot transform from " + target_posestamped.header.frame_id \
                + " to " + self.base_frame_id + ".")

        target_pos, target_quat = rosPose2PosQuat(target_posestamped_in_base.pose)
        target_rotvec = sciR.from_quat(target_quat).as_rotvec()

        self.rtde_c.moveL(np.concatenate([target_pos, target_rotvec]), 
            speed=speed, acceleration=acceleration, asynchronous=asynchronous) 
        





# --------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    try:
        rospy.init_node("ur_node")
        ur = URrtde("192.168.100.50")

    except rospy.ROSInterruptException:
        pass