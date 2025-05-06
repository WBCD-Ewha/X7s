import open3d as o3d
import numpy as np
import cv2
import argparse

# 1. 파일 경로 설정
rgb_path = '../datasets/sample_0/subtask1/camera_h/image_left.png'
depth_path = '../datasets/sample_0/subtask1/camera_h/depth_image_raw16.png'

parser = argparse.ArgumentParser()
parser.add_argument('--path', type=str, required=True)
parser.add_argument('--output', type=str, required=True,
                    help="Output point cloud file path")
args = parser.parse_args()


path = args.path
output_path = args.output

# 2. 이미지 불러오기
rgb = cv2.imread(rgb_path)
depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)  # uint16
mask = cv2.imread(path, cv2.IMREAD_GRAYSCALE)    # mask는 grayscale (0:배경, 255:객체)

# 3. Depth 카메라 intrinsic
depth_intrinsic = {
    "fx": 394.7567443847656,
    "fy": 394.7567443847656,
    "cx": 321.34881591796875,
    "cy": 240.4374542236328,
    "width": 640,
    "height": 480
}

# 4. Color 카메라 intrinsic
color_intrinsic = {
    "fx": 385.76702880859375,
    "fy": 385.0887145996094,
    "cx": 327.74560546875,
    "cy": 244.9661102294922,
    "width": 640,
    "height": 480
}

# 5. Depth to Color Extrinsic
R = np.array([
    [0.9999993, -0.0011171, -0.0002352],
    [0.0011170, 0.9999993, -0.0004513],
    [0.0002357, 0.0004511, 0.9999999]
])
t = np.array([-0.05923, -0.000169, 0.0002484])

# 6. 포인트 클라우드 복원
points = []
colors = []
depth_scale = 1000.0  # mm to meters

for v in range(depth.shape[0]):
    for u in range(depth.shape[1]):
        d = depth[v, u] / depth_scale
        if d == 0 or d > 2.5:
            continue

        # Depth 카메라 기준 3D 복원
        x = (u - depth_intrinsic['cx']) * d / depth_intrinsic['fx']
        y = (v - depth_intrinsic['cy']) * d / depth_intrinsic['fy']
        z = d
        p_depth = np.array([x, y, z])

        # Depth -> Color extrinsic 적용
        p_color = R @ p_depth + t

        # Color 카메라로 투영
        u_color = int((p_color[0] * color_intrinsic['fx']) / p_color[2] + color_intrinsic['cx'])
        v_color = int((p_color[1] * color_intrinsic['fy']) / p_color[2] + color_intrinsic['cy'])

        if 0 <= u_color < color_intrinsic['width'] and 0 <= v_color < color_intrinsic['height']:
            # (u_color, v_color) 위치에서 mask 확인
            if mask[v_color, u_color] == 0:
                continue  # mask가 0(배경) 이면 skip

            color = rgb[v_color, u_color, :] / 255.0  # RGB 색상
            points.append(p_depth)
            colors.append(color)

# 7. PointCloud 생성
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(np.array(points))
pcd.colors = o3d.utility.Vector3dVector(np.array(colors))

# 8. 시각화 함수 정의
def custom_draw_geometry(pcd):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()

    ctr = vis.get_view_control()
    ctr.set_front([0.0, 0.0, -1.0])
    ctr.set_lookat([0.0, 0.0, 0.0])
    ctr.set_up([0.0, -1.0, 0.0])
    ctr.set_zoom(0.5)

    vis.run()
    vis.destroy_window()

o3d.io.write_point_cloud(output_path, pcd)

# 9. 시각화 호출
custom_draw_geometry(pcd)

