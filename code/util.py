import numpy as np
import open3d as o3d

container_open_path = '../datasets/sample_0/subtask1/camera_h/image_left.png'
container_open_depth_path  = '../datasets/sample_0/subtask1/camera_h/depth_image_raw16.png'
container_open_mask_path = '../../Grounded-Segment-Anything/outputs/container_0.jpg'

pizza_grasp_path = '../datasets/sample_0/subtask1/camera_h/image_left.png'
pizza_grasp_depth_path = '../datasets/sample_0/subtask1/camera_h/depth_image_raw16.png'
pizza_grasp_plate_path = '../../Grounded-Segment-Anything/outputs/plate_0.jpg'
pizza_grasp_container_path = '../../Grounded-Segment-Anything/outputs/container_1.jpg'

container_close_path = '../datasets/sample_0/subtask2/camera_l/image_left.png'
container_close_depth_path = '../datasets/sample_0/subtask2/camera_l/depth_image_raw16.png'
container_close_lid_path = '../../Grounded-Segment-Anything/outputs/container_lid.jpg'
container_close_container_path = '../../Grounded-Segment-Anything/outputs/container_2.jpg'

# Intrinsic 파라미터
depth_K = {
    "fx": 394.7567443847656, "fy": 394.7567443847656,
    "cx": 321.34881591796875, "cy": 240.4374542236328,
    "width": 640, "height": 480
}
color_K= {
    "fx": 385.76702880859375, "fy": 385.0887145996094,
    "cx": 327.74560546875, "cy": 244.9661102294922,
    "width": 640, "height": 480
}
R = np.array([
    [0.9999993, -0.0011171, -0.0002352],
    [0.0011170, 0.9999993, -0.0004513],
    [0.0002357, 0.0004511, 0.9999999]
])
t = np.array([-0.05923, -0.000169, 0.0002484])

def transform_matrix(point: np.ndarray):
    """3D point to 4x4 matrix"""
    T = np.eye(4)
    T[:3, 3] = point
    return T

def create_coordinate_frame(T, size=0.02):
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size)
    return frame.transform(T)

def create_sphere(pos, color=[0, 0, 1], radius=0.007):
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
    sphere.paint_uniform_color(color)
    sphere.translate(pos)
    return sphere

def pca(points: np.ndarray):
    """
    Perform PCA on a set of 3D points to extract the primary (long) and secondary (short) axes.
    """
    center = np.mean(points, axis=0)
    centered = points - center
    _, _, Vt = np.linalg.svd(centered)
    long_axis = Vt[0]
    short_axis = Vt[1]
    return long_axis, short_axis