#!/usr/bin/python

import rospy
import numpy as np
from scipy.spatial.transform import Rotation as sciR
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from ur_rtde_msgs.msg import VectorStamped
from moveit_msgs.msg import RobotTrajectory


from ur_rtde import URrtde
from dual_ur_rtde import DualURrtde
from utils import *




# --------------------------------------------------------------------------------
class DualURrtdeNode():
    def __init__(self):
        self.camera_frame_id = "rgb_camera_link"
        self.world_frame_id = "world"

        # left arm
        ur_0_home_pose = None
        ur_0_tcp_offset_trans = rospy.get_param("robot_configs/arm_0/tcp_in_ee_pose/position")
        ur_0_tcp_offset_rotvec = sciR.from_quat(rospy.get_param("robot_configs/arm_0/tcp_in_ee_pose/orientation")).as_rotvec()
        ur_0_tcp_offset = ur_0_tcp_offset_trans + ur_0_tcp_offset_rotvec.tolist()
        arm_0 = URrtde("192.168.100.50", prefix="arm_0_", home_pose=ur_0_home_pose, 
            tcp_offset=ur_0_tcp_offset, base_frame_id="arm_0_base", world_frame_id="world")

        # right arm
        ur_1_home_pose = None
        ur_1_tcp_offset_trans = rospy.get_param("robot_configs/arm_1/tcp_in_ee_pose/position")
        ur_1_tcp_offset_rotvec = sciR.from_quat(rospy.get_param("robot_configs/arm_1/tcp_in_ee_pose/orientation")).as_rotvec()
        ur_1_tcp_offset = ur_1_tcp_offset_trans + ur_1_tcp_offset_rotvec.tolist()
        arm_1 = URrtde("192.168.101.50", prefix="arm_1_", home_pose=ur_1_home_pose, 
            tcp_offset=ur_1_tcp_offset, base_frame_id="arm_1_base", world_frame_id="world")
        
        self.dual_arm = DualURrtde(arm_0, arm_1)

        rospy.Subscriber("control/dual_arm/joint_vel_command", VectorStamped, 
            self.dualArmJointVelCommandCb)
        rospy.Subscriber("control/arm_0/joint_trajectory_command", RobotTrajectory, 
            self.arm0JointTrajCommandCb)
        rospy.Subscriber("control/arm_1/joint_trajectory_command", RobotTrajectory, 
            self.arm1JointTrajCommandCb)
        rospy.Subscriber("control/dual_arm/joint_trajectory_command", RobotTrajectory, 
            self.dualArmJointTrajCommandCb)

        self.ros_rate = 30
        # self.dual_arm_joint_state_pub = rospy.Publisher("state/dual_arm/joint_states", JointState, queue_size=1)
        self.dual_arm_joint_state_pub = rospy.Publisher("state/real_time/dual_arm/joint_states", JointState, queue_size=1)
        self.arm_0_tcp_pose_in_world_pub = rospy.Publisher("state/real_time/arm_0/tcp_pose_in_world", PoseStamped, queue_size=1)
        self.arm_1_tcp_pose_in_world_pub = rospy.Publisher("state/real_time/arm_1/tcp_pose_in_world", PoseStamped, queue_size=1)
        self.arm_0_tcp_pose_in_cam_pub = rospy.Publisher("state/real_time/arm_0/tcp_pose_in_cam", PoseStamped, queue_size=1)
        self.arm_1_tcp_pose_in_cam_pub = rospy.Publisher("state/real_time/arm_1/tcp_pose_in_cam", PoseStamped, queue_size=1)

        # self.dual_arm_move_joint_pos_srv = rospy.Service("control/dual_arm/joint_pos_command", 
        #     ExecuteToJointPosService, self.dualArmToJointPos)

        # self.dual_arm_move_end_pose_srv = rospy.Service("control/dual_arm/end_pose_command", 
        #     ExecuteToEndPose, self.dualArmToEndPose)

        # self.ros_rate = rospy.get_param("ros_rate/arm_rate")


    # -----------------------------------------------------------------
    def dualArmJointVelCommandCb(self, msg):
        dual_arm_joint_vel = msg.data

        print(dual_arm_joint_vel)

        if len(dual_arm_joint_vel) != 12:
            rospy.logerr("Invalid dual_arm_joint_vel command, the size is %d !", len(dual_arm_joint_vel))

        self.dual_arm.arm_0.controlJointVel(dual_arm_joint_vel[:6])
        self.dual_arm.arm_1.controlJointVel(dual_arm_joint_vel[6:])

    
    # -----------------------------------------------------------------
    def arm0JointTrajCommandCb(self, msg):
        self.dual_arm.arm_0.moveJointTrajectory(msg)

    # -----------------------------------------------------------------
    def arm1JointTrajCommandCb(self, msg):
        self.dual_arm.arm_1.moveJointTrajectory(msg)

    # -----------------------------------------------------------------
    def dualArmJointTrajCommandCb(self, msg):
        self.dual_arm.moveJointTrajectory(msg)


    # # -----------------------------------------------------------------
    # def dualArmToJointPos(self, req):
    #     dual_arm_joint_pos = req.target_joint_pos
    #     self.ur_0.rtde_c.moveL_FK(dual_arm_joint_pos[:6], speed=0.1, acceleration=0.5, asynchronous=True) 
    #     self.ur_1.rtde_c.moveL_FK(dual_arm_joint_pos[6:], speed=0.1, acceleration=0.5, asynchronous=True) 

    #     # 等待达到目标位置
    #     rate = rospy.Rate(self.ros_rate)
    #     target_dual_arm_joint_pos = np.array(dual_arm_joint_pos)
    #     rospy.loginfo("Moving to the target dual arm joint pos...")
    #     while not rospy.is_shutdown():
    #         current_dual_arm_joint_pos = np.array(self.ur_0.getJointAngle() + self.ur_1.getJointAngle())
    #         # print("current_dual_arm_joint_pos: ", current_dual_arm_joint_pos)
    #         # print("target_dual_arm_joint_pos: ", target_dual_arm_joint_pos)
    #         if np.linalg.norm(current_dual_arm_joint_pos - target_dual_arm_joint_pos) < 1e-3:
    #             rospy.loginfo("Done.")
    #             break
    #         rate.sleep()

    #     return ExecuteToJointPosServiceResponse(True)


    # # -----------------------------------------------------------------
    # def dualArmToEndPose(self, req):
    #     self.ur_0.moveToTcpPose(req.end_0_pose, speed=0.1, acceleration=0.5, asynchronous=True)
    #     self.ur_1.moveToTcpPose(req.end_1_pose, speed=0.1, acceleration=0.5, asynchronous=True)

    #     # 等待达到目标位置
    #     target_end_0_pos, target_end_0_quat = rosPose2PosQuat(req.end_0_pose.pose)
    #     target_end_1_pos, target_end_1_quat = rosPose2PosQuat(req.end_1_pose.pose)
    #     rate = rospy.Rate(self.ros_rate)
    #     rospy.loginfo("Moving to the target dual arm end pose...")
    #     while not rospy.is_shutdown():
    #         current_end_0_posestamped = self.ur_0.getTcpPose("dual_base")
    #         current_end_1_posestamped = self.ur_1.getTcpPose("dual_base")
    #         current_end_0_pos, current_end_0_quat = rosPose2PosQuat(current_end_0_posestamped.pose)
    #         current_end_1_pos, current_end_1_quat = rosPose2PosQuat(current_end_1_posestamped.pose)

    #         if np.linalg.norm(target_end_0_pos - current_end_0_pos) < 1e-3 \
    #                 and np.linalg.norm(target_end_0_quat - current_end_0_quat) < 1e-3 \
    #                 and np.linalg.norm(target_end_1_pos - current_end_1_pos) < 1e-3 \
    #                 and np.linalg.norm(target_end_1_quat - current_end_1_quat) < 1e-3:
    #             rospy.loginfo("Done.")
    #             break
    #         rate.sleep()

    #     return ExecuteToEndPoseResponse(True)
    


    # -----------------------------------------------------------------
    def main(self):
        rospy.loginfo("dual_ur_rtde_node starts.")
        # 打印初始时的关节角
        print("arm_0 initial joint angle: ", self.dual_arm.arm_0.getJointAngle())
        print("arm_1 initial joint angle: ", self.dual_arm.arm_1.getJointAngle())

        rate = rospy.Rate(30)

        while not rospy.is_shutdown():

            # 发布 joint state
            self.dual_arm_joint_state_pub.publish(self.dual_arm.getDualArmJointState())
            # rospy.logdebug("UR rtde: current joint pos 0: %.5f", self.dual_arm.getDualArmJointState().position[0])

            # 发布 TCP poses in world
            arm_0_tcp_posestamped = self.dual_arm.arm_0.getTcpPose(target_frame_id=self.world_frame_id)
            arm_1_tcp_posestamped = self.dual_arm.arm_1.getTcpPose(target_frame_id=self.world_frame_id)
            self.arm_0_tcp_pose_in_world_pub.publish(arm_0_tcp_posestamped)
            self.arm_1_tcp_pose_in_world_pub.publish(arm_1_tcp_posestamped)

            arm_0_tcp_posestamped = self.dual_arm.arm_0.getTcpPose(target_frame_id=self.camera_frame_id)
            arm_1_tcp_posestamped = self.dual_arm.arm_1.getTcpPose(target_frame_id=self.camera_frame_id)
            self.arm_0_tcp_pose_in_cam_pub.publish(arm_0_tcp_posestamped)
            self.arm_1_tcp_pose_in_cam_pub.publish(arm_1_tcp_posestamped)

            rate.sleep()


        # shutdown the robots
        self.dual_arm.robotShutDown()





# --------------------------------------------------------------
if __name__ == "__main__":
    try:
        rospy.init_node("dual_ur_rtde_node", log_level=rospy.DEBUG)
        
        dual_ur = DualURrtdeNode()
        dual_ur.main()


    except rospy.ROSInterruptException:
        pass