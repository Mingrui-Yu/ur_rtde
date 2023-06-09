#!/usr/bin/python3

"""
- 用于 URrtde 接收和发送 message / service 的接口
- 根据任务需要进行自定义
"""

import rospy
import numpy as np
from ur_rtde import URrtde
from scipy.spatial.transform import Rotation as sciR

from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from ur_rtde_msgs.msg import VectorStamped


# --------------------------------------------------------------------------------
class URrtdeNode():
    def __init__(self):
        ur_home_config = [-1.5952280203448694, -1.5429447332965296, -1.3991649786578577, -1.774517838154928, 1.639392614364624, -0.09030753770937139]
        ur_tcp_offset_trans = [0, 0, 0]
        ur_tcp_offset_rotvec = [0, 0, 0]
        ur_tcp_offset = ur_tcp_offset_trans + ur_tcp_offset_rotvec
        self.ur = URrtde("192.168.100.50", prefix="", home_pose=ur_home_config, 
            tcp_offset=ur_tcp_offset, base_frame_id="base", world_frame_id="world")
        
        self.ros_rate = 30
        
        rospy.Subscriber("/control/arm/joint_vel_command", VectorStamped, self.armJointVelCommandCb)
        rospy.Subscriber("/control/arm/tcp_vel_in_world_command", TwistStamped, self.armTcpVelInWorldCommandCb)
        
        
    # -----------------------------------------------------------------
    def armJointVelCommandCb(self, msg):
        joint_vel = msg.data
        self.ur.controlJointVel(joint_vel, acceleration=2.0)
        
        
    # -----------------------------------------------------------------
    def armTcpVelInWorldCommandCb(self, msg):
        self.ur.controlTcpVel(msg, acceleration=1.0)


    # -----------------------------------------------------------------
    def main(self):
        rospy.loginfo("ur_rtde_node starts.")
        
        # 打印初始时的关节角
        print("arm initial joint angle: ", self.ur.getJointAngle())

        rate = rospy.Rate(self.ros_rate)

        while not rospy.is_shutdown():
            rate.sleep()


        # shutdown the robots
        self.ur.robotShutDown()





# --------------------------------------------------------------
if __name__ == "__main__":
    try:
        rospy.init_node("ur_rtde_node")
        
        ur_node = URrtdeNode()
        ur_node.main()


    except rospy.ROSInterruptException:
        pass