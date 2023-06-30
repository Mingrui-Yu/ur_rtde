from scipy.spatial.transform import Rotation as sciR
from scipy.linalg import sqrtm
import numpy as np


# ------------------------
R_base_in_baselink = sciR.from_euler('XYZ', [0, 0, 180], degrees=True)
T_base_in_baselink = np.zeros((4, 4))
T_base_in_baselink[0:3, 0:3] = (R_base_in_baselink).as_matrix()
T_base_in_baselink[3, 3] = 1

t_cam_in_base0 = np.array([-0.8028931100595065, -0.08612737095294531, -0.38702159879302406])
R_cam_in_base0 = sciR.from_quat([0.6417433448292816, 0.2946866337158286, 0.3166235826818352, 0.6333046456218981])

t_cam_in_base1 = np.array([0.8131619997695416, -0.09393868137364425, -0.314799536180339])
R_cam_in_base1 = sciR.from_quat([-0.3163314168218155, -0.6437165907936334,  0.6347226816757491, 0.2875595639773968])

T_cam_in_base0 = np.zeros((4, 4))
T_cam_in_base0[0:3, 0:3] = (R_cam_in_base0).as_matrix()
T_cam_in_base0[0:3, 3] = t_cam_in_base0.reshape(-1,)
T_cam_in_base0[3, 3] = 1

T_cam_in_baselink0 = T_base_in_baselink @ T_cam_in_base0

T_cam_in_base1 = np.zeros((4, 4))
T_cam_in_base1[0:3, 0:3] = (R_cam_in_base1).as_matrix()
T_cam_in_base1[0:3, 3] = t_cam_in_base1.reshape(-1,)
T_cam_in_base1[3, 3] = 1

T_cam_in_baselink1 = T_base_in_baselink @ T_cam_in_base1

T_base1_in_base0 = T_cam_in_base0 @ np.linalg.inv(T_cam_in_base1)

T_baselink1_in_baselink0 = T_cam_in_baselink0 @ np.linalg.inv(T_cam_in_baselink1)

print("T_baselink1_in_baselink0: \n", T_baselink1_in_baselink0)
print("euler_baselink1_in_baselink0: \n", sciR.from_matrix(T_baselink1_in_baselink0[0:3, 0:3]).as_euler('xyz', degrees=False))
print("t_baselink1_in_baselink0: \n", T_baselink1_in_baselink0[0:3, 3])


t_baselink0_in_world = np.array([0, -0.357, 0])
R_baselink0_in_world = sciR.from_euler('xyz', [90, 0, 0], degrees=True)
T_baselink0_in_world = np.zeros((4, 4))
T_baselink0_in_world[0:3, 0:3] = R_baselink0_in_world.as_matrix()
T_baselink0_in_world[0:3, 3] = t_baselink0_in_world.reshape(-1,)
T_baselink0_in_world[3, 3] = 1

T_baselink1_in_world = T_baselink0_in_world @ T_baselink1_in_baselink0

print("T_baselink1_in_world: \n", T_baselink1_in_world)
print("euler_baselink1_in_world: \n", sciR.from_matrix(T_baselink1_in_world[0:3, 0:3]).as_euler('xyz', degrees=False))
print("t_baselink1_in_world: \n", T_baselink1_in_world[0:3, 3])



# R_base0_in_world = R_base0_in_world * sciR.from_euler('XYZ', [2, 0, 0], degrees=True)

# t_base1_in_world = np.array([0, 0.357, 0])
# R_base1_in_world = sciR.from_euler('xyz', [-90, 180, 0], degrees=True)
# R_base1_in_world = R_base1_in_world * sciR.from_euler('xyz', [2, 0, 0], degrees=True)

# T_base0_in_world = np.zeros((4, 4))
# T_base0_in_world[0:3, 0:3] = R_base0_in_world.as_matrix()
# T_base0_in_world[0:3, 3] = t_base0_in_world.reshape(-1,)
# T_base0_in_world[3, 3] = 1

# T_base1_in_world = np.zeros((4, 4))
# T_base1_in_world[0:3, 0:3] = R_base1_in_world.as_matrix()
# T_base1_in_world[0:3, 3] = t_base1_in_world.reshape(-1,)
# T_base1_in_world[3, 3] = 1

# T_baselink1_in_baselink0 = np.linalg.inv(T_base0_in_world) @ T_base1_in_world

# print("T_baselink1_in_baselink0: \n", T_baselink1_in_baselink0)
# print("euler_baselink1_in_baselink0: \n", sciR.from_matrix(T_baselink1_in_baselink0[0:3, 0:3]).as_euler('xyz', degrees=False))
# print("t_baselink1_in_baselink0: \n", T_baselink1_in_baselink0[0:3, 3])

# print("euler_base0_in_world: \n", sciR.from_matrix(T_base0_in_world[0:3, 0:3]).as_euler('xyz', degrees=False))
# print("t_base0_in_world: \n", T_base0_in_world[0:3, 3])

# print("euler_base1_in_world: \n", sciR.from_matrix(T_base1_in_world[0:3, 0:3]).as_euler('xyz', degrees=False))
# print("t_base1_in_world: \n", T_base1_in_world[0:3, 3])
