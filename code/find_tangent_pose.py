import numpy as np
import cv2
import open3d as o3d
from color3dmapper import ColorTo3DMapper

# 1. Load pixel coordinates
grasp_point = np.load("grasp_point.npy")  # (x, y)
intersection_point = np.load("intersection_point.npy")  # edge
entry_point = np.load("entry_point.npy")

container_height = 0.4

# 2. Camera parameters
depth_path = "../datasets/sample_0/subtask1/camera_h/depth_image_raw16.png"
rgb_path = "../datasets/sample_0/subtask1/camera_h/image_left.png"

depth_K = {"fx": 394.7567, "fy": 394.7567, "cx": 321.3488, "cy": 240.4375}
color_K = {"fx": 385.767, "fy": 385.089, "cx": 327.746, "cy": 244.966}
R = np.array([
    [0.9999993, -0.0011171, -0.0002352],
    [0.0011170,  0.9999993, -0.0004513],
    [0.0002357,  0.0004511,  0.9999999]
])
t = np.array([-0.05923, -0.000169, 0.0002484])

# 3. Instantiate mapper and project to 3D
mapper = ColorTo3DMapper(depth_path, rgb_path, depth_K, color_K, R, t)
grasp_3d, _ = mapper.get_3d_point(grasp_point)
entry_3d, _ = mapper.get_3d_point(entry_point)
edge_3d, _ = mapper.get_3d_point(intersection_point)

# 4. 이동 벡터
move_vec = edge_3d - entry_3d
new_grasp_xy = grasp_3d + move_vec

new_grasp_pose = np.array([new_grasp_xy[0], new_grasp_xy[1], container_height])

# 5. Grasp pose 만들기 (기본 z축: 아래로)
grasp_x = move_vec / np.linalg.norm(move_vec)
grasp_z = np.array([0, 0, -1])
grasp_y = np.cross(grasp_z, grasp_x)
grasp_y /= np.linalg.norm(grasp_y)

grasp_pose = np.eye(4)
grasp_pose[:3, 0] = grasp_x
grasp_pose[:3, 1] = grasp_y
grasp_pose[:3, 2] = grasp_z
grasp_pose[:3, 3] = new_grasp_pose

moved_entry_xy = entry_3d + move_vec
moved_entry_pose = np.array([moved_entry_xy[0], moved_entry_xy[1], container_height])

np.save("grasp_pose_transformed.npy", grasp_pose)

# 6. Point cloud 생성 및 시각화
pcd = mapper.get_open3d_pointcloud()  # Open3D point cloud 객체 반환된다고 가정

# 7. Grasp pose 시각화용 좌표축
def create_coordinate_frame(T, size=0.02):
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size)
    return frame.transform(T)

grasp_frame = create_coordinate_frame(grasp_pose, size=0.05)

entry_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
entry_sphere.paint_uniform_color([1.0, 0.6, 0.0])  # 주황색
entry_sphere.translate(moved_entry_pose)


o3d.visualization.draw_geometries([pcd, grasp_frame, entry_sphere], window_name="Grasp Pose in 3D")