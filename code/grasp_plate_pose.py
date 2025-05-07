import numpy as np
import open3d as o3d
from color3dmapper import ColorTo3DMapper
from extrinsic import apply_extrinsic_to_point, apply_extrinsic_to_pose_matrix

# 1. 파일 경로
rgb_path = '../datasets/sample_0/subtask1/camera_h/image_left.png'
depth_path = '../datasets/sample_0/subtask1/camera_h/depth_image_raw16.png'
grasp_uv_color = np.load("grasp_point.npy")  # (u, v) in color image

# 2. 카메라 파라미터
depth_K = {"fx": 394.7567, "fy": 394.7567, "cx": 321.3488, "cy": 240.4375}
color_K = {"fx": 385.767, "fy": 385.089, "cx": 327.746, "cy": 244.966}
R = np.array([
    [0.9999993, -0.0011171, -0.0002352],
    [0.0011170,  0.9999993, -0.0004513],
    [0.0002357,  0.0004511,  0.9999999]
])
t = np.array([-0.05923, -0.000169, 0.0002484])

# 3. 매퍼 객체 생성
mapper = ColorTo3DMapper(
    depth_path=depth_path,
    rgb_path=rgb_path,
    depth_K=depth_K,
    color_K=color_K,
    R=R,
    t=t,
    depth_scale=1000.0
)

# 4. grasp point에 해당하는 3D 좌표 얻기
grasp_point_3d, _ = mapper.get_3d_point(grasp_uv_color)

# 5. Open3D point cloud 생성
pcd = mapper.get_open3d_pointcloud()

# 6. grasp 포인트 시각화
grasp_marker = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
grasp_marker.paint_uniform_color([1, 0, 0])
grasp_marker.translate(grasp_point_3d)

# 7. 시각화
o3d.visualization.draw_geometries([pcd, grasp_marker])

# extrinsic
grasp_point_3d = apply_extrinsic_to_point(grasp_point_3d)
