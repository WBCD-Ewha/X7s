import open3d as o3d
import numpy as np

# 1. Load PCD
pcd = o3d.io.read_point_cloud("container_pt.ply")
points = np.asarray(pcd.points)

# 2. 바닥 평면 검출 (RANSAC)
plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                         ransac_n=3,
                                         num_iterations=1000)
[a, b, c, d] = plane_model
normal = np.array([a, b, c])
normal /= np.linalg.norm(normal)  # 단위 벡터로 정규화

# 3. 바닥 평면인지 확인
if np.dot(normal, np.array([0, 0, 1])) < 0.85:
    raise RuntimeError("No flat bottom plane detected.")

# 4. 바닥 중심 계산
bottom_points = points[inliers]
bottom_center = np.mean(bottom_points, axis=0)

# 5. PCA 수행 (surface 방향 분석)
center = np.mean(bottom_points, axis=0)
centered = bottom_points - center
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

# 5. 바닥 제외한 나머지 upper 영역 추출
non_bottom_indices = list(set(range(points.shape[0])) - set(inliers))
upper_points = points[non_bottom_indices]
upper_center = np.mean(upper_points, axis=0)

# 6. upper_center를 bottom_center로부터 normal 방향으로 투영
vec = upper_center - bottom_center
height_along_normal = np.dot(vec, normal)

# 7. 최종 뚜껑 중심 계산
lid_center = bottom_center + normal * height_along_normal
print("Estimated lid center (projected along normal):", lid_center)

# 8. gripper 위치를 뚜껑 높이만큼 올리기
offset_vec = normal * height_along_normal
left_pose[:3, 3] += offset_vec
right_pose[:3, 3] += offset_vec

print("Left grasp pose:\n", left_pose)
print("Right grasp pose:\n", right_pose)

# 9. 시각화용 finger marker 생성
def create_sphere(pos, color=[0, 0, 1], radius=0.007):
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
    sphere.paint_uniform_color(color)
    sphere.translate(pos)
    return sphere

finger_left = create_sphere(left_pose[:3, 3], color=[0, 0, 1])
finger_right = create_sphere(right_pose[:3, 3], color=[0, 0, 1])

# 10. 시각화
upper_pcd = pcd.select_by_index(non_bottom_indices)
upper_pcd.paint_uniform_color([1.0, 0.0, 0.0])

lid_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
lid_sphere.paint_uniform_color([0, 1, 0])
lid_sphere.translate(lid_center)

pcd.paint_uniform_color([0.7, 0.7, 0.7])

o3d.visualization.draw_geometries([
    pcd, upper_pcd, lid_sphere,
    finger_left, finger_right
])