import numpy as np
from scipy.spatial.transform import Rotation as sciR
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory


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



# ----------------------------------------------------------------------
def trajectoryInterpolation(trajectory, dt):
    n_waypoints = len(trajectory.joint_trajectory.points)
    interpolated_traj = RobotTrajectory()
    t = 0.0
    for i in range(n_waypoints - 1):
        current_point = trajectory.joint_trajectory.points[i]
        next_point = trajectory.joint_trajectory.points[i+1]
        t0 = current_point.time_from_start.to_sec()
        t1 = next_point.time_from_start.to_sec()

        while t < t1:
            point = JointTrajectoryPoint()
            x0 = np.array(current_point.positions)
            v0 = np.array(current_point.velocities)
            x1 = np.array(next_point.positions)
            v1 = np.array(next_point.velocities)

            if np.all(np.array(v0) == 0) or np.all(np.array(v1) == 0):
                v = (x1 - x0) / (t1 - t0)
                point.velocities = v.tolist()
                point.positions = (x0 + v * (t - t0)).tolist()
            else:
                av = (v1 - v0) / (t1 - t0)
                ax = 2.0 / ((t1 - t0)*(t1 - t0)) * ((x1 - x0) - v0 * (t1 - t0))
                point.velocities = (v0 + av * (t - t0)).tolist()
                point.positions = (x0 + v0 * (t - t0) + ax / 2.0 * (t - t0) * (t - t0)).tolist()
            
            interpolated_traj.joint_trajectory.points.append(point)
            t += dt

    interpolated_traj.joint_trajectory.points.append(trajectory.joint_trajectory.points[-1])  # 最后，加上原trajectory的destination
    interpolated_traj.joint_trajectory.header = trajectory.joint_trajectory.header

    return interpolated_traj