import open3d as o3d
import numpy as np

# 1. PointCloud 로드
pcd = o3d.io.read_point_cloud("container_pt.ply")

# 2. 평면 검출 (RANSAC 기반)
plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                         ransac_n=3,
                                         num_iterations=1000)
[a, b, c, d] = plane_model
normal = np.array([a, b, c])
normal /= np.linalg.norm(normal)

# 3. Top surface 확인
dot = np.dot(normal, np.array([0, 0, 1]))
if dot < 0.85:
    raise RuntimeError("No top surface detected.")

print("Top surface detected.")

# 4. Top surface points 추출
top_surface_points = np.asarray(pcd.points)[inliers]

# 5. PCA 수행 (surface 방향 분석)
center = np.mean(top_surface_points, axis=0)
centered = top_surface_points - center
_, _, Vt = np.linalg.svd(centered)

long_axis = Vt[0]  # 가장 긴 방향
short_axis = Vt[1]  # 짧은 방향 (gripper 방향)

# 6. Gripper frame 설정
gripper_y = short_axis / np.linalg.norm(short_axis)
gripper_x = long_axis / np.linalg.norm(long_axis)
gripper_z = np.cross(gripper_x, gripper_y)
gripper_z /= np.linalg.norm(gripper_z)

# 7. Hinge offset만큼 이동해서 finger 위치 결정
hinge_offset = 0.06  # 3cm 이동 (네가 원하는 값)

left_pose = np.eye(4)
left_pose[:3, 0] = gripper_x
left_pose[:3, 1] = gripper_y
left_pose[:3, 2] = gripper_z
left_pose[:3, 3] = center - gripper_y * hinge_offset

right_pose = np.eye(4)
right_pose[:3, 0] = gripper_x
right_pose[:3, 1] = gripper_y
right_pose[:3, 2] = gripper_z
right_pose[:3, 3] = center + gripper_y * hinge_offset

print("Left grasp pose:\n", left_pose)
print("Right grasp pose:\n", right_pose)

# 8. Visualization 함수
def create_arrow(start, direction, length=0.05, color=[1, 0, 0]):
    arrow = o3d.geometry.TriangleMesh.create_arrow(
        cylinder_radius=0.002,
        cone_radius=0.004,
        cylinder_height=length * 0.8,
        cone_height=length * 0.2
    )
    arrow.paint_uniform_color(color)
    z = np.array([0, 0, 1])
    axis = np.cross(z, direction)
    if np.linalg.norm(axis) > 1e-6:
        angle = np.arccos(np.clip(np.dot(z, direction), -1.0, 1.0))
        R = o3d.geometry.get_rotation_matrix_from_axis_angle(axis / np.linalg.norm(axis) * angle)
        arrow.rotate(R, center=(0, 0, 0))
    arrow.translate(start)
    return arrow

def create_sphere(pos, color=[0,1,0], radius=0.007):
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
    sphere.paint_uniform_color(color)
    sphere.translate(pos)
    return sphere

# 9. 시각화 준비
arrow_x = create_arrow(center, gripper_x, color=[0, 0, 1])
arrow_y = create_arrow(center, gripper_y, color=[1, 0, 0])
arrow_z = create_arrow(center, gripper_z, color=[0, 1, 0])

finger_left = create_sphere(left_pose[:3, 3], color=[0, 0, 1])
finger_right = create_sphere(right_pose[:3, 3], color=[0, 0, 1])
center_sphere = create_sphere(center)

top_surface_pcd = pcd.select_by_index(inliers)
top_surface_pcd.paint_uniform_color([1.0, 0.0, 0.0])
pcd.paint_uniform_color([0.7, 0.7, 0.7])

o3d.visualization.draw_geometries([
    pcd, top_surface_pcd,
    finger_left, finger_right, center_sphere,
    arrow_x, arrow_y, arrow_z
])