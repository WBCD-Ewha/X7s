import numpy as np
import open3d as o3d
import cv2
from color3dmapper import ColorTo3DMapper

# --- 1. Load saved 2D centers ---
lid_center_2d = np.load("lid_center.npy")          # shape: (2,)
container_center_2d = np.load("container2_center.npy")  # shape: (2,)

# --- 2. Load depth image and intrinsic ---
rgb_path = '../datasets/sample_0/subtask2/camera_l/image_left.png'
depth_path = '../datasets/sample_0/subtask2/camera_l/depth_image_raw16.png'

lid_mask = cv2.imread("../../Grounded-Segment-Anything/outputs/container_lid.jpg", cv2.IMREAD_GRAYSCALE)
container_mask = cv2.imread("../../Grounded-Segment-Anything/outputs/container_2.jpg", cv2.IMREAD_GRAYSCALE)

depth_K = {"fx": 394.7567, "fy": 394.7567, "cx": 321.3488, "cy": 240.4375}
color_K = {"fx": 385.767, "fy": 385.089, "cx": 327.746, "cy": 244.966}
R = np.array([
    [0.9999993, -0.0011171, -0.0002352],
    [0.0011170,  0.9999993, -0.0004513],
    [0.0002357,  0.0004511,  0.9999999]
])
t = np.array([-0.05923, -0.000169, 0.0002484])

mapper = ColorTo3DMapper(
    depth_path=depth_path,
    rgb_path=rgb_path,
    depth_K=depth_K,
    color_K=color_K,
    R=R,
    t=t,
    depth_scale=1000.0
)

# --- 3. Convert 2D centers to 3D ---
lid_center_3d, _ = mapper.get_3d_point(lid_center_2d)
container_center_3d, _ = mapper.get_3d_point(container_center_2d)

if lid_center_3d is None or container_center_3d is None:
    raise ValueError("Depth 값이 존재하지 않습니다.")

# --- 4. Load original grasp points in 3D (from findpose_lid.py) ---
left_grasp = np.load("align_left_grasp.npy")     # shape: (3,)
right_grasp = np.load("align_right_grasp.npy")   # shape: (3,)
# --- 5. Compute translation vector and apply it to grasp points ---
translation = container_center_3d - lid_center_3d
left_grasp_translated = left_grasp + translation
right_grasp_translated = right_grasp + translation

# --- 6. Print results ---
print("Original Lid Center (3D):", lid_center_3d)
print("Target Container Center (3D):", container_center_3d)
print("Translation Vector:", translation)
print("\nTranslated Left Grasp Point:", left_grasp_translated)
print("Translated Right Grasp Point:", right_grasp_translated)

# --- 7. Optional visualization (Open3D) ---
full_pcd = mapper.get_open3d_pointcloud()
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector([lid_center_3d, container_center_3d, left_grasp, right_grasp, left_grasp_translated, right_grasp_translated])
pcd.colors = o3d.utility.Vector3dVector([
    [0, 0, 1],   # lid center
    [0, 1, 0],   # container center
    [1, 0, 0],   # original left
    [1, 0.5, 0], # original right
    [0, 1, 1],   # translated left
    [1, 0, 1]    # translated right
])

lines = o3d.geometry.LineSet(
    points=o3d.utility.Vector3dVector([
        left_grasp, right_grasp,
        left_grasp_translated, right_grasp_translated
    ]),
    lines=o3d.utility.Vector2iVector([
        [0, 1], [2, 3]
    ])
)
lines.colors = o3d.utility.Vector3dVector([[1, 0, 0], [0, 1, 0]])

o3d.visualization.draw_geometries([full_pcd, pcd, lines])