from scipy.spatial.transform import Rotation as sciR
import numpy as np

# tool in end

# left
rot = sciR.from_euler('XYZ', [180, 0, 0], degrees=True)
print("rotvec: ", rot.as_rotvec())
print("quat(x, y, z, w) : ", rot.as_quat())

# right
rot = sciR.from_euler('XYZ', [180, 0, 180], degrees=True)
print("rotvec: ", rot.as_rotvec())
print("quat(x, y, z, w) : ", rot.as_quat())





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