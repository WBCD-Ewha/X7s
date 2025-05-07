import numpy as np

R = np.array([
    [0.9998, 0.0176, -0.0012],
    [-0.0176, 0.9997, -0.0153],
    [0.0015, 0.0153, 0.9999]
])
t = np.array([0.12, 0.04, 0.89])

def apply_extrinsic_to_point(point):
    """3D point에 extrinsic transform 적용"""
    return R @ point + t

def apply_extrinsic_to_pose_matrix(pose_mat):
    """4x4 pose matrix에 extrinsic transform 적용"""
    T_extrinsic = np.eye(4)
    T_extrinsic[:3, :3] = R
    T_extrinsic[:3, 3] = t
    return T_extrinsic @ pose_mat