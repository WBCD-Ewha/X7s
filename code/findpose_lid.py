import numpy as np
import cv2
import open3d as o3d
from extrinsic import apply_extrinsic_to_point, apply_extrinsic_to_pose_matrix
import argparse

# ---- 파일 경로 ----
mask_path = '../../Grounded-Segment-Anything/outputs/container_lid.jpg'
rgb_path = '../datasets/sample_0/subtask2/camera_l/image_left.png'
depth_path = '../datasets/sample_0/subtask2/camera_l/depth_image_raw16.png'

# ---- 카메라 Intrinsic & Extrinsic ----
depth_K = {'fx': 394.7567, 'fy': 394.7567, 'cx': 321.3488, 'cy': 240.4375}
color_K = {'fx': 385.767, 'fy': 385.089, 'cx': 327.746, 'cy': 244.966}
R = np.array([[0.9999993, -0.0011171, -0.0002352],
              [0.0011170,  0.9999993, -0.0004513],
              [0.0002357,  0.0004511,  0.9999999]])
t = np.array([-0.05923, -0.000169, 0.0002484])
depth_scale = 1000.0

# ---- 이미지 로딩 ----
mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
rgb = cv2.imread(rgb_path)  # BGR
rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED).astype(np.float32)

# ---- 마스크 영역의 픽셀을 3D로 변환 ----
def pixel_to_3d(u, v, depth_img, K, R, t):
    z = depth_img[v, u] / depth_scale
    if z == 0: return None
    x = (u - K['cx']) * z / K['fx']
    y = (v - K['cy']) * z / K['fy']
    p_d = np.array([x, y, z])
    p_c = R @ p_d + t  # ← color 카메라 좌표계로 변환
    return p_c

points_3d = []
colors = []
for v in range(mask.shape[0]):
    for u in range(mask.shape[1]):
        if mask[v, u] > 0:
            pt = pixel_to_3d(u, v, depth, depth_K, R, t)
            if pt is not None:
                points_3d.append(pt)
                color = rgb[v, u] / 255.0  # normalize to [0,1]
                colors.append(color)

points_3d = np.array(points_3d)
colors = np.array(colors)

if len(points_3d) < 10:
    raise RuntimeError("3D 점이 너무 적습니다. 마스크 또는 depth 데이터를 확인하세요.")

# ---- PCA 및 Grasp 계산 (기존과 동일) ----
center = points_3d.mean(axis=0)
points_centered = points_3d - center
_, _, Vt = np.linalg.svd(points_centered, full_matrices=False)
short_axis = Vt[1]

hinge_offset = 0.06  # meters
left_grasp = center - short_axis * hinge_offset
right_grasp = center + short_axis * hinge_offset

# ---- 시각화 ----
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points_3d)
pcd.colors = o3d.utility.Vector3dVector(colors)

grasp_points = o3d.geometry.PointCloud()
grasp_points.points = o3d.utility.Vector3dVector([center, left_grasp, right_grasp])
grasp_points.colors = o3d.utility.Vector3dVector([[0, 1, 0], [1, 0, 0], [0, 0, 1]])

grasp_line = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector([left_grasp, right_grasp]),
    lines=o3d.utility.Vector2iVector([[0, 1]])
)
grasp_line.colors = o3d.utility.Vector3dVector([[1, 0, 0]])

print("Left Grasp Point (red):", left_grasp)
print("Right Grasp Point (blue):", right_grasp)

np.save("align_left_grasp.npy", left_grasp)
np.save("align_right_grasp.npy", right_grasp)

o3d.visualization.draw_geometries([pcd, grasp_points, grasp_line])

#extrinsic
left_pose = apply_extrinsic_to_point(left_grasp)
right_pose = apply_extrinsic_to_point(right_grasp)


