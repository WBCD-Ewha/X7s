import open3d as o3d
import numpy as np
import random

# 1. Plate PointCloud 로드
pcd = o3d.io.read_point_cloud("plate_pt.ply")  # <- 여기 경로 맞춰줘

# 2. PointCloud의 중심(center) 계산
points = np.asarray(pcd.points)
center = np.mean(points, axis=0)

# 3. 중심으로부터 각 점까지 거리 계산
distances = np.linalg.norm(points - center, axis=1)

# 4. 가장자리 후보 포인트 필터링
# (중심에서 거리가 큰 점들만 선택)
threshold = np.percentile(distances, 90)  # 상위 10% 거리
edge_indices = np.where(distances > threshold)[0]

edge_points = points[edge_indices]

# 5. 가장자리 포인트 중 랜덤으로 하나 선택
chosen_idx = random.choice(range(edge_points.shape[0]))
chosen_point = edge_points[chosen_idx]

print(f"Selected grasp point: {chosen_point}")

# 6. Grasp pose 생성 (edge → 중심 방향을 y축으로)
grasp_pose = np.eye(4)

# 중심 방향 벡터 (edge → center)
y_axis = center - chosen_point
y_axis /= np.linalg.norm(y_axis)

# 6-1. offset 만큼 더 안쪽으로 들어간 grasp point 계산
offset = 0.03  # 3cm 정도 안쪽으로 들어감 (원하는 만큼 조정 가능)
grasp_position = chosen_point + y_axis * offset  # edge → center 방향으로 이동


# 임의의 기준 벡터 (z축과 거의 수직한 평면상의 x축 후보)
temp = np.array([0, 0, 1])
if abs(np.dot(temp, y_axis)) > 0.9:
    temp = np.array([1, 0, 0])  # y축과 너무 평행하면 다른 기준 사용

x_axis = np.cross(y_axis, temp)
x_axis /= np.linalg.norm(x_axis)

z_axis = np.cross(x_axis, y_axis)
z_axis /= np.linalg.norm(z_axis)

grasp_pose[:3, 0] = x_axis
grasp_pose[:3, 1] = y_axis
grasp_pose[:3, 2] = z_axis

# 6-2. 최종 Grasp Pose 업데이트
grasp_pose[:3, 3] = grasp_position

print("Grasp pose (edge → center):\n", grasp_pose)

# 7. 시각화
def create_sphere(pos, color=[0,1,0], radius=0.01):
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
    sphere.paint_uniform_color(color)
    sphere.translate(pos)
    return sphere

grasp_sphere = create_sphere(grasp_position, color=[0, 1, 0])  # offset 적용된 grasp 위치

# 중심 시각화
center_sphere = create_sphere(center, color=[1, 0, 0], radius=0.008)

o3d.visualization.draw_geometries([
    pcd, grasp_sphere, center_sphere
])