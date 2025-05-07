import open3d as o3d

# PLY 파일 경로
ply_path = "../datasets/sample_0/subtask1/camera_h/point_cloud.ply"

# PLY 파일 읽기
pcd = o3d.io.read_point_cloud(ply_path)

# 시각화 창 생성
vis = o3d.visualization.Visualizer()
vis.create_window(window_name="Closer PLY Viewer", width=800, height=600, left=50, top=50)
vis.add_geometry(pcd)

# 뷰 설정
view_ctl = vis.get_view_control()
view_ctl.translate(18, 0)

view_ctl.set_zoom(0.1)  # 숫자가 작을수록 더 가까이. 기본은 0.7 정도

# 렌더링 & 보여주기
vis.run()
vis.destroy_window()