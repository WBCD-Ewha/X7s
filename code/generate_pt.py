import open3d as o3d
import numpy as np
import cv2

# 1. 이미지 불러오기
rgb = o3d.io.read_image('../datasets/sample_0/subtask1/camera_l/image_left_cv.png')
d = o3d.io.read_image('../datasets/sample_0/subtask1/camera_l/depth_image_raw16.png')

rgb_np = np.asarray(rgb)
depth_np = np.asarray(d)

# 2. Depth 카메라 intrinsic
depth_intrinsic = {
    "fx": 394.7567443847656,
    "fy": 394.7567443847656,
    "cx": 321.34881591796875,
    "cy": 240.4374542236328,
    "width": 640,
    "height": 480
}

# 3. Color 카메라 intrinsic
color_intrinsic = {
    "fx": 385.76702880859375,
    "fy": 385.0887145996094,
    "cx": 327.74560546875,
    "cy": 244.9661102294922,
    "width": 640,
    "height": 480
}

# 4. Depth → Color Extrinsic
R = np.array([
    [0.9999993, -0.0011171, -0.0002352],
    [0.0011170, 0.9999993, -0.0004513],
    [0.0002357, 0.0004511, 0.9999999]
])
t = np.array([-0.05923, -0.000169, 0.0002484])

# 5. 포인트 복원 (depth 카메라 기준)
points = []
colors = []

depth_scale = 1000.0  # depth 값이 mm 단위라면

depth_cutoff = 1.0  # 단위: meters

for v in range(depth_np.shape[0]):
    for u in range(depth_np.shape[1]):
        d = depth_np[v, u] / depth_scale  # mm → meters
        if d == 0 or d > depth_cutoff:
            continue
        # depth 카메라 frame 3D point (X, Y, Z)
        x = (u - depth_intrinsic['cx']) * d / depth_intrinsic['fx']
        y = (v - depth_intrinsic['cy']) * d / depth_intrinsic['fy']
        z = d
        p_depth = np.array([x, y, z])

        # 6. Depth frame → Color frame 변환
        p_color = R @ p_depth + t

        # 7. Color 카메라에 투영해서 (u,v) 얻기
        u_color = int((p_color[0] * color_intrinsic['fx']) / p_color[2] + color_intrinsic['cx'])
        v_color = int((p_color[1] * color_intrinsic['fy']) / p_color[2] + color_intrinsic['cy'])

        # 8. Color 이미지 범위 체크
        if 0 <= u_color < color_intrinsic['width'] and 0 <= v_color < color_intrinsic['height']:
            color = rgb_np[v_color, u_color, :] / 255.0  # Normalize to [0,1]
            points.append(p_depth)
            colors.append(color)


# 9. Open3D PointCloud 생성
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(np.array(points))
pcd.colors = o3d.utility.Vector3dVector(np.array(colors))

# 10. 시각화
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

custom_draw_geometry(pcd)
