from cloth_tools.annotation.grasp_annotation import grasp_hanging_cloth_pose

from generate_pt import load_images, generate_point_cloud
import open3d as o3d
import cv2
import numpy as np

from util import (
    transform_matrix, create_coordinate_frame, create_sphere, pca,
    container_open_path, container_open_depth_path, container_open_mask_path,
    depth_K, color_K, R, t
)

def visualize(pcd, upper_pcd, lid_center, left_pose, right_pose):

    lid_sphere = create_sphere(lid_center, color=[0, 1, 0], radius=0.01)
    left_grasp_frame = create_coordinate_frame(left_pose, size=0.05)
    right_grasp_frame = create_coordinate_frame(right_pose, size=0.05)


    # 색상 지정
    upper_pcd.paint_uniform_color([1.0, 0.0, 0.0])  # RED
    pcd.paint_uniform_color([0.7, 0.7, 0.7])        # GRAY

    print("Left Grasp Point (red):\n", left_pose)
    print("Right Grasp Point (blue):\n", right_pose)

    # 시각화
    o3d.visualization.draw_geometries([
        pcd, upper_pcd, lid_sphere,
        left_grasp_frame, right_grasp_frame
    ])

def estimate_lid_grasp_poses(pcd: o3d.geometry.PointCloud, hinge_offset=0.06):
    """
    Estimate left/right gripper poses to grasp the lid of a container.
    """
    points = np.asarray(pcd.points)

    # 1. extract bottom plane
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                             ransac_n=3,
                                             num_iterations=1000)
    [a, b, c, d] = plane_model
    normal = np.array([a, b, c])
    normal /= np.linalg.norm(normal)

    # 2. verify plane
    if np.dot(normal, np.array([0, 0, 1])) < 0.85:
        raise RuntimeError("No flat bottom plane detected.")

    # 3. center of bottom
    bottom_points = points[inliers]
    bottom_center = np.mean(bottom_points, axis=0)

    # 4. calculate directio vector using pca
    long_axis, short_axis = pca(bottom_points)

    # 5. gripper frame
    gripper_y = short_axis / np.linalg.norm(short_axis)
    gripper_x = long_axis / np.linalg.norm(long_axis)
    gripper_z = np.cross(gripper_x, gripper_y)
    gripper_z /= np.linalg.norm(gripper_z)

    # 6. 바닥 기준 finger 위치 (중심에서 hinge_offset 만큼 이동)
    left_point = bottom_center - gripper_y * hinge_offset
    right_point = bottom_center + gripper_y * hinge_offset

    # 7. lid 높이 계산
    non_bottom_indices = list(set(range(points.shape[0])) - set(inliers))
    upper_points = points[non_bottom_indices]
    upper_center = np.mean(upper_points, axis=0)
    height_along_normal = np.dot(upper_center - bottom_center, normal)

    # 8. lid 중심
    lid_center = bottom_center + normal * height_along_normal

    # 9. gripper point를 lid 높이만큼 위로 이동
    offset_vec = normal * height_along_normal
    left_point += offset_vec
    right_point += offset_vec

    # 10. grasp pose 생성 (아래 → 위 방향)
    approach = np.array([0, 0, 1])
    left_pose = grasp_hanging_cloth_pose(left_point, approach)
    right_pose = grasp_hanging_cloth_pose(right_point, approach)

    # 10. 시각화 실행
    upper_pcd = pcd.select_by_index(non_bottom_indices)

    visualize(pcd, upper_pcd, lid_center, left_pose, right_pose)

def main():
    rgb, depth, mask = load_images(container_open_path, container_open_depth_path, container_open_mask_path)
    pcd = generate_point_cloud(rgb, depth, mask, depth_K, color_K, R, t)

    estimate_lid_grasp_poses(pcd, hinge_offset = 0.06)

if __name__ == "__main__":
    main()