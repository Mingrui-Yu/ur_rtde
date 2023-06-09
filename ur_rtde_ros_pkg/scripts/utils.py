import numpy as np
from geometry_msgs.msg import Pose


# -------------------------------------------------------
def rosPose2PosQuat(pose):
    pos = np.array([pose.position.x, pose.position.y, pose.position.z])
    # x,y,z,w
    quat = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    return pos, quat