from scipy.spatial.transform import Rotation as sciR
import numpy as np

# tool in end

# # left
# rot = sciR.from_euler('XYZ', [180, 0, 0], degrees=True)
# print("rotvec: ", rot.as_rotvec())
# print("quat(x, y, z, w) : ", rot.as_quat())

# # right
# rot = sciR.from_euler('XYZ', [180, 0, 180], degrees=True)
# print("rotvec: ", rot.as_rotvec())
# print("quat(x, y, z, w) : ", rot.as_quat())


# mat = np.array([[1.0, 0.0044, 0.0046], 
#                 [-0.0048, 0.9945, 0.1043],
#                 [-0.0042, -0.1043, 0.9945]])

# quat = sciR.from_matrix(mat.T).as_quat()
# print(quat)


# ------------------------
pos = np.array([0.8094910280448541, -0.08949178336109319, -0.31444344853554307])
quat = np.array([-0.3178628514852475 ,-0.6434592738530964 ,0.6341495954464997, 0.28771107231588117])

R = sciR.from_quat(quat).as_matrix()

T = np.zeros((4, 4))
T[0:3, 0:3] = R
T[0:3, 3] = pos.reshape(-1,)
T[3, 3] = 1

T_inv = np.linalg.inv(T)
R_inv = T_inv[0:3, 0:3]
quat_inv = sciR.from_matrix(R_inv).as_quat()
t_inv = T_inv[0:3, 3]

print(t_inv)
print(quat_inv)



# # simulation
# rot_mat = sciR.from_quat([0.707106781, 0.707106781, 0.0, 0.0]).as_matrix()
# trans = np.array([0.0, 0.0, 0.145]).reshape(-1, 1)

# transform_matrix = np.block([[rot_mat, trans], [np.zeros((1, 3)), 1.0]])

# transform_matrix_inv = np.linalg.inv(transform_matrix)
# quat_inv = sciR.from_matrix(transform_matrix_inv[0:3, 0:3]).as_quat()
# trans_inv = transform_matrix_inv[0:3, 3]

# print("quat_inv: ", quat_inv)
# print("trans_inv: ", trans_inv)
# # 取逆后没有变化