from generate_pt import load_images, generate_point_cloud, generate_point_cloud_2, load_images_2
import rospy
import open3d as o3d
import cv2
import numpy as np
from util import (
    transform_matrix, create_coordinate_frame, create_sphere, pca,
    container_close_path, container_close_depth_path, container_close_lid_path, container_close_container_path,
    depth_K, color_K, R, t
)
from ros_sender import send_bimanual_pose, send_bimanual_close_pose

# remove outlier
def cut_outlier(pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
    iqr_multiplier = 2.0  # 낮출수록 많이 잘려나감

    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)

    # 열별로 IQR를 계산합니다.
    q1 = np.percentile(points, 25, axis=0)
    q3 = np.percentile(points, 75, axis=0)
    iqr = q3 - q1

    # 각 열에 대한 하한과 상한을 계산합니다.
    lower_bound = q1 - iqr_multiplier * iqr
    upper_bound = q3 + iqr_multiplier * iqr

    # 각 열에서 아웃라이어를 식별합니다.
    outlier_mask = (points < lower_bound) | (points > upper_bound)

    # 행 중에서 모든 열이 아웃라이어가 아닌 경우를 찾아 데이터에서 제거합니다.
    points = points[~np.any(outlier_mask, axis=1)]
    colors = colors[~np.any(outlier_mask, axis=1)]

    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    return pcd

def visualize_close_3d(pcd, container_center_3d, lid_center_3d, new_left_grasp, new_right_grasp):
    new_left_grasp = transform_matrix(new_left_grasp)
    new_right_grasp = transform_matrix(new_right_grasp)
    left_grasp_frame = create_coordinate_frame(new_left_grasp, size=0.05)
    right_grasp_frame = create_coordinate_frame(new_right_grasp, size=0.05)

    container_sphere = create_sphere(container_center_3d, color=[0, 0, 1])
    lid_sphere = create_sphere(lid_center_3d, color=[1, 0, 0])

    print("Left Grasp Point (red):\n", new_left_grasp)
    print("Right Grasp Point (blue):\n", new_right_grasp)

    o3d.visualization.draw_geometries(
        [pcd, container_sphere, lid_sphere, left_grasp_frame, right_grasp_frame])

def visualize_3d(pcd, container_center_3d, lid_center_3d, left_grasp, right_grasp, long_axis, short_axis):
    left_grasp = transform_matrix(left_grasp)
    right_grasp = transform_matrix(right_grasp)
    left_grasp_frame = create_coordinate_frame(left_grasp, size=0.05)
    right_grasp_frame = create_coordinate_frame(right_grasp, size=0.05)

    container_sphere = create_sphere(container_center_3d, color=[0, 0, 1])
    lid_sphere = create_sphere(lid_center_3d, color=[1, 0, 0])

    axis_length = 0.1

    long_line = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector([
            lid_center_3d,
            lid_center_3d + long_axis * axis_length
        ]),
        lines=o3d.utility.Vector2iVector([[0, 1]])
    )
    long_line.colors = o3d.utility.Vector3dVector([[1, 0, 0]])  # 빨강

    short_line = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector([
            lid_center_3d,
            lid_center_3d + short_axis * axis_length
        ]),
        lines=o3d.utility.Vector2iVector([[0, 1]])
    )
    short_line.colors = o3d.utility.Vector3dVector([[0, 1, 0]])  # 초록

    print("Left Grasp Point (red):\n", left_grasp)
    print("Right Grasp Point (blue):\n", right_grasp)

    o3d.visualization.draw_geometries([pcd, container_sphere, lid_sphere, left_grasp_frame, right_grasp_frame, long_line, short_line])

def get_pcd_center(pcd: o3d.geometry.PointCloud) -> np.ndarray:
    points = np.asarray(pcd.points)
    if points.shape[0] == 0:
        raise ValueError("PointCloud is empty. Cannot compute center.")
    center = np.mean(points, axis=0)
    return center

def align_grasp_pose(grasp1, grasp2):
    if grasp1[0] < grasp2[0]:
        return grasp1, grasp2 # left right
    else:
        return grasp2, grasp1

def find_lid_grasp_pose(pcd, lid_center_3d, hinge_offset):
    # TODO: solve the problem of long and short axis
    points = np.asarray(pcd.points)
    long_axis, short_axis = pca(points)

    # gripper frame
    gripper_y = short_axis / np.linalg.norm(short_axis)
    gripper_x = long_axis / np.linalg.norm(long_axis)
    gripper_z = np.cross(gripper_x, gripper_y)
    gripper_z /= np.linalg.norm(gripper_z)

    left_grasp = lid_center_3d - gripper_y * hinge_offset
    right_grasp = lid_center_3d + gripper_y * hinge_offset

    left_grasp, right_grasp = align_grasp_pose(left_grasp, right_grasp)

    return left_grasp, right_grasp, long_axis, short_axis

def close_lid_grasp(left_grasp, right_grasp, lid_center_3d, container_center_3d, container_height=0.4):
    move_vec = container_center_3d - lid_center_3d
    new_left_grasp_xy = left_grasp + move_vec
    new_right_grasp_xy = right_grasp + move_vec
    new_left_grasp_pose = np.array([
        new_left_grasp_xy[0], new_left_grasp_xy[1], container_height
    ])
    new_right_grasp_pose = np.array([
        new_right_grasp_xy[0], new_right_grasp_xy[1], container_height
    ])

    new_left_grasp_pose, new_right_grasp_pose = align_grasp_pose(new_left_grasp_pose, new_right_grasp_pose)

    return new_left_grasp_pose, new_right_grasp_pose

def main():
    rospy.init_node("container_pose_estimator")

    rgb, depth, mask1, mask2 = load_images_2(container_close_path, container_close_depth_path, container_close_container_path, container_close_lid_path)

    pcd = generate_point_cloud_2(rgb, depth, mask1, mask2, depth_K, color_K, R, t)
    pcd = cut_outlier(pcd)
    pcd_lid= generate_point_cloud(rgb, depth, mask2, depth_K, color_K, R, t)
    pcd_container = generate_point_cloud(rgb, depth, mask1, depth_K, color_K, R, t)

    lid_center_3d = get_pcd_center(pcd_lid)
    container_center_3d = get_pcd_center(pcd_container)

    left_grasp, right_grasp, long_axis, short_axis = find_lid_grasp_pose(pcd, lid_center_3d, hinge_offset=0.06)
    visualize_3d(pcd, container_center_3d, lid_center_3d, left_grasp, right_grasp, long_axis, short_axis)

    send_bimanual_pose(left_grasp, right_grasp)

    rospy.sleep(2.0)

    new_left_grasp, new_right_grasp = close_lid_grasp(left_grasp, right_grasp, lid_center_3d, container_center_3d)
    visualize_close_3d(pcd, container_center_3d, lid_center_3d, new_left_grasp, new_right_grasp)

    send_bimanual_close_pose(new_left_grasp, new_right_grasp)

if __name__ == "__main__":
    main()