import numpy as np
import cv2
import open3d as o3d
import argparse

# ---- 파일 경로 ----
mask_path = '../../Grounded-Segment-Anything/outputs/container_lid.jpg'
rgb_path = '../datasets/sample_0/subtask1/camera_h/image_left.png'
depth_path = '../datasets/sample_0/subtask1/camera_h/depth_image_raw16.png'

# ---- 카메라 Intrinsic & Extrinsic ----
depth_K = {'fx': 394.7567, 'fy': 394.7567, 'cx': 321.3488, 'cy': 240.4375}
color_K = {'fx': 385.767, 'fy': 385.089, 'cx': 327.746, 'cy': 244.966}
R = np.array([[0.9999993, -0.0011171, -0.0002352],
              [0.0011170,  0.9999993, -0.0004513],
              [0.0002357,  0.0004511,  0.9999999]])
t = np.array([-0.05923, -0.000169, 0.0002484])
depth_scale = 1000.0  # mm → m

# ---- 이미지 로딩 ----
mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
rgb = cv2.imread(rgb_path)
depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED).astype(np.float32)

# ---- 1. Segmentation에서 손잡이 contour 추출 ----
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
contours = sorted(contours, key=cv2.contourArea, reverse=True)[:2]

if len(contours) < 2:
    raise RuntimeError("손잡이 contour 2개가 필요합니다. 현재: {}".format(len(contours)))

# ---- 2. 손잡이 중심점 추출 ----
centers = []
for cnt in contours:
    M = cv2.moments(cnt)
    if M['m00'] == 0: continue
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    centers.append((cx, cy))  # Color 이미지 기준 좌표

p1, p2 = np.array(centers[0]), np.array(centers[1])
grasp_center = ((p1 + p2) / 2).astype(int)
grasp_dir = p2 - p1
grasp_dir = grasp_dir / np.linalg.norm(grasp_dir)
grasp_normal = np.array([-grasp_dir[1], grasp_dir[0]])

# ---- 3. grasp 좌/우 포인트 계산 ----
offset = 20  # pixel
left_px = grasp_center + (grasp_normal * offset).astype(int)
right_px = grasp_center - (grasp_normal * offset).astype(int)

# ---- 4. 픽셀 → Color 카메라 좌표계 3D 포인트 ----
def pixel_to_3d_color(u, v, depth_img, color_K):
    z = depth_img[v, u] / depth_scale
    if z == 0: return None
    x = (u - color_K['cx']) * z / color_K['fx']
    y = (v - color_K['cy']) * z / color_K['fy']
    return np.array([x, y, z])

# ---- 5. Color → Depth 좌표계로 변환 ----
def transform_color_to_depth(p_c, R, t):
    return np.linalg.inv(R) @ (p_c - t)

# Get 3D points (color frame)
center_3d_color = pixel_to_3d_color(grasp_center[0], grasp_center[1], depth, color_K)
left_3d_color = pixel_to_3d_color(left_px[0], left_px[1], depth, color_K)
right_3d_color = pixel_to_3d_color(right_px[0], right_px[1], depth, color_K)

if None in [center_3d_color, left_3d_color, right_3d_color]:
    raise ValueError("Grasp point 중 하나 이상에서 depth 정보가 없습니다.")

# Convert to depth frame (optional, depending on use case)
center_3d_depth = transform_color_to_depth(center_3d_color, R, t)
left_3d_depth = transform_color_to_depth(left_3d_color, R, t)
right_3d_depth = transform_color_to_depth(right_3d_color, R, t)

# ---- 6. 시각화 및 저장 ----
line = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector([left_3d_depth, right_3d_depth]),
    lines=o3d.utility.Vector2iVector([[0, 1]])
)
line.colors = o3d.utility.Vector3dVector([[1, 0, 0]])

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector([center_3d_depth, left_3d_depth, right_3d_depth])
pcd.colors = o3d.utility.Vector3dVector([[0, 1, 0]] * 3)

# o3d.io.write_point_cloud(output_path, pcd)
o3d.visualization.draw_geometries([line, pcd])
