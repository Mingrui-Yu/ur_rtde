#!/usr/bin/python

import rospy
import numpy as np
from ur_rtde import URrtde
from scipy.spatial.transform import Rotation as sciR
from utils import *

from sensor_msgs.msg import JointState
from ur_rtde_msgs.msg import VectorStamped
# from my_msgs.srv import ExecuteToJointPosService, ExecuteToJointPosServiceResponse
# from my_msgs.srv import ExecuteToEndPose, ExecuteToEndPoseResponse


# --------------------------------------------------------------------------------
class DualURrtde():
    def __init__(self):
        # left arm
        ur_0_home_pose = [-3.768959347401754, -2.5454705397235315, -1.4650076071368616, -2.263789955769674, -0.6454971472369593, -0.7995479742633265]
        ur_0_tcp_offset_trans = rospy.get_param("robot/arm_0/tool_in_end/translation")
        ur_0_tcp_offset_rotvec = sciR.from_quat(rospy.get_param("robot/arm_0/tool_in_end/rotation")).as_rotvec()
        ur_0_tcp_offset = ur_0_tcp_offset_trans + ur_0_tcp_offset_rotvec.tolist()
        self.ur_0 = URrtde("192.168.100.50", prefix="arm_0_", home_pose=ur_0_home_pose, 
            tcp_offset=ur_0_tcp_offset, base_frame_id="arm_0_base", world_frame_id="dual_base")

        # right arm
        ur_1_home_pose = [-2.5164807478534144, -0.6666477362262171, 1.4899320602416992, -0.8363164106952112, 0.6629456281661987, 0.7799333930015564]
        ur_1_tcp_offset_trans = rospy.get_param("robot/arm_1/tool_in_end/translation")
        ur_1_tcp_offset_rotvec = sciR.from_quat(rospy.get_param("robot/arm_1/tool_in_end/rotation")).as_rotvec()
        ur_1_tcp_offset = ur_1_tcp_offset_trans + ur_1_tcp_offset_rotvec.tolist()
        self.ur_1 = URrtde("192.168.101.50", prefix="arm_1_", home_pose=ur_1_home_pose, 
            tcp_offset=ur_1_tcp_offset, base_frame_id="arm_1_base", world_frame_id="dual_base")


        self.dual_arm_joint_state_pub = rospy.Publisher("state/dual_arm/joint_states", JointState, queue_size=1)

        rospy.Subscriber("control/dual_arm/joint_vel_command", VectorStamped, 
            self.dualArmJointVelCommandCb)

        self.dual_arm_move_joint_pos_srv = rospy.Service("control/dual_arm/joint_pos_command", 
            ExecuteToJointPosService, self.dualArmToJointPos)

        self.dual_arm_move_end_pose_srv = rospy.Service("control/dual_arm/end_pose_command", 
            ExecuteToEndPose, self.dualArmToEndPose)

        self.ros_rate = rospy.get_param("ros_rate/arm_rate")


    # -----------------------------------------------------------------
    def dualArmJointVelCommandCb(self, msg):
        dual_arm_joint_vel = msg.data
        self.ur_0.rtde_c.speedJ(dual_arm_joint_vel[:6], acceleration=2.0, time=0.001)
        self.ur_1.rtde_c.speedJ(dual_arm_joint_vel[6:], acceleration=2.0, time=0.001)


    # -----------------------------------------------------------------
    def dualArmToJointPos(self, req):
        dual_arm_joint_pos = req.target_joint_pos
        self.ur_0.rtde_c.moveL_FK(dual_arm_joint_pos[:6], speed=0.1, acceleration=0.5, asynchronous=True) 
        self.ur_1.rtde_c.moveL_FK(dual_arm_joint_pos[6:], speed=0.1, acceleration=0.5, asynchronous=True) 

        # 等待达到目标位置
        rate = rospy.Rate(self.ros_rate)
        target_dual_arm_joint_pos = np.array(dual_arm_joint_pos)
        rospy.loginfo("Moving to the target dual arm joint pos...")
        while not rospy.is_shutdown():
            current_dual_arm_joint_pos = np.array(self.ur_0.getJointAngle() + self.ur_1.getJointAngle())
            # print("current_dual_arm_joint_pos: ", current_dual_arm_joint_pos)
            # print("target_dual_arm_joint_pos: ", target_dual_arm_joint_pos)
            if np.linalg.norm(current_dual_arm_joint_pos - target_dual_arm_joint_pos) < 1e-3:
                rospy.loginfo("Done.")
                break
            rate.sleep()

        return ExecuteToJointPosServiceResponse(True)


    # -----------------------------------------------------------------
    def dualArmToEndPose(self, req):
        self.ur_0.moveToTcpPose(req.end_0_pose, speed=0.1, acceleration=0.5, asynchronous=True)
        self.ur_1.moveToTcpPose(req.end_1_pose, speed=0.1, acceleration=0.5, asynchronous=True)

        # 等待达到目标位置
        target_end_0_pos, target_end_0_quat = rosPose2PosQuat(req.end_0_pose.pose)
        target_end_1_pos, target_end_1_quat = rosPose2PosQuat(req.end_1_pose.pose)
        rate = rospy.Rate(self.ros_rate)
        rospy.loginfo("Moving to the target dual arm end pose...")
        while not rospy.is_shutdown():
            current_end_0_posestamped = self.ur_0.getTcpPose("dual_base")
            current_end_1_posestamped = self.ur_1.getTcpPose("dual_base")
            current_end_0_pos, current_end_0_quat = rosPose2PosQuat(current_end_0_posestamped.pose)
            current_end_1_pos, current_end_1_quat = rosPose2PosQuat(current_end_1_posestamped.pose)

            if np.linalg.norm(target_end_0_pos - current_end_0_pos) < 1e-3 \
                    and np.linalg.norm(target_end_0_quat - current_end_0_quat) < 1e-3 \
                    and np.linalg.norm(target_end_1_pos - current_end_1_pos) < 1e-3 \
                    and np.linalg.norm(target_end_1_quat - current_end_1_quat) < 1e-3:
                rospy.loginfo("Done.")
                break
            rate.sleep()

        return ExecuteToEndPoseResponse(True)
        

    # -----------------------------------------------------------------
    def getDualArmJointState(self):
        joint_state = JointState()
        joint_state.name = self.ur_0.joint_names + self.ur_1.joint_names
        joint_state.position = self.ur_0.getJointAngle() + self.ur_1.getJointAngle()
        joint_state.velocity = self.ur_0.getJointVel() + self.ur_1.getJointVel()

        return joint_state


    # -----------------------------------------------------------------
    def publishDualArmJointState(self):
        self.dual_arm_joint_state_pub.publish(self.getDualArmJointState())


    # -----------------------------------------------------------------
    def main(self):

        rospy.loginfo("dual_ur_rtde_node starts.")
        # 打印初始时的关节角
        print("arm_0 initial joint angle: ", self.ur_0.getJointAngle())
        print("arm_1 initial joint angle: ", self.ur_1.getJointAngle())

        rate = rospy.Rate(self.ros_rate)

        while not rospy.is_shutdown():

            self.publishDualArmJointState()

            rate.sleep()


        # shutdown the robots
        self.ur_0.robotShutDown()
        self.ur_1.robotShutDown()





# --------------------------------------------------------------
if __name__ == "__main__":
    try:
        rospy.init_node("dual_ur_rtde_node")
        
        dual_ur = DualURrtde()
        dual_ur.main()


    except rospy.ROSInterruptException:
        pass