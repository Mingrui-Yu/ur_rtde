import numpy as np
from scipy.spatial.transform import Rotation as sciR
from geometry_msgs.msg import Pose


# -------------------------------------------------------
def rosPose2PosQuat(pose):
    pos = np.array([pose.position.x, pose.position.y, pose.position.z])
    # x,y,z,w
    quat = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    return pos, quat


# -------------------------------------------------------
def distanceBetweenRosPoses(pose_0, pose_1, pos_weight=1.0, rot_weight=1.0):
    pos_0, quat_0 = rosPose2PosQuat(pose_0)
    pos_1, quat_1 = rosPose2PosQuat(pose_1)

    pos_dist = np.linalg.norm(pos_0 - pos_1)
    rot_dist = sciR.magnitude(sciR.from_quat(quat_1) * sciR.from_quat(quat_0).inv())

    return pos_weight * pos_dist + rot_weight * rot_dist
