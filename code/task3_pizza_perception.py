from generate_pt import generate_point_cloud, generate_point_cloud_2, load_images_2
import open3d as o3d
import cv2
import numpy as np
from util import (
    transform_matrix, create_coordinate_frame, create_sphere,
    pizza_grasp_path, pizza_grasp_depth_path, pizza_grasp_plate_path, pizza_grasp_container_path,
    depth_K, color_K, R, t
)
from mapper import point_mapper


def find_center(mask: np.ndarray):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        raise ValueError("No contour found in mask.")
    largest_contour = max(contours, key=cv2.contourArea)
    M = cv2.moments(largest_contour)
    if M["m00"] == 0:
        raise ZeroDivisionError("Contour area is zero.")
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    return np.array([cx, cy]), largest_contour

def compute_intersection(p1: np.ndarray, p2: np.ndarray, q1: np.ndarray, q2: np.ndarray):
    A = np.array([[p2[0] - p1[0], q1[0] - q2[0]],
                  [p2[1] - p1[1], q1[1] - q2[1]]])
    b = np.array([q1[0] - p1[0], q1[1] - p1[1]])

    if np.linalg.det(A) == 0:
        return None  # 평행

    t, s = np.linalg.solve(A, b)
    if 0 <= s <= 1:  # q 선분 내부일 때만 유효
        return p1 + t * (p2 - p1)
    return None


def line_intersects_contour(p1: np.ndarray, p2: np.ndarray, contour: np.ndarray):
    """
    선분 p1-p2가 contour 경계와 교차하는 모든 점 반환
    """
    intersections = []
    for i in range(len(contour)):
        a1 = contour[i][0]
        a2 = contour[(i + 1) % len(contour)][0]
        inter_pt = compute_intersection(p1, p2, a1, a2)
        if inter_pt is not None:
            intersections.append(inter_pt)
    return intersections

def visualize_2d(
    raw_image: np.ndarray,
    plate_center: np.ndarray,
    container_center: np.ndarray,
    grasp_point: np.ndarray,
    nearest_point: np.ndarray,
    radius: float,
    intersection_pt: np.ndarray = None
):
    vis = raw_image.copy()

    cv2.circle(vis, tuple(plate_center), int(radius), (255, 0, 255), 2)          # 접시 원
    cv2.circle(vis, tuple(plate_center), 5, (0, 0, 255), -1)                      # 접시 중심
    cv2.circle(vis, tuple(container_center), 5, (0, 255, 0), -1)                  # 컨테이너 중심
    cv2.circle(vis, tuple(grasp_point.astype(int)), 7, (255, 255, 0), -1)         # grasp 위치
    cv2.circle(vis, tuple(nearest_point.astype(int)), 7, (0, 255, 255), -1)       # entry 위치
    cv2.arrowedLine(vis, tuple(grasp_point.astype(int)), tuple(container_center.astype(int)),
                    (0, 255, 255), 2, tipLength=0.2)                              # grasp → container 방향

    if intersection_pt is not None and (intersection_pt >= 0).all():
        cv2.circle(vis, tuple(intersection_pt), 6, (0, 165, 255), -1)             # 주황색 교차점

    cv2.imshow("Grasp Planning", vis)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def visualize_3d(pcd, mapped_point):
    grasp_pose = transform_matrix(mapped_point)
    grasp_pose_frame = create_coordinate_frame(grasp_pose, size=0.05)

    print("Grasp Point :\n", grasp_pose)

    # 시각화
    o3d.visualization.draw_geometries([
        pcd, grasp_pose_frame
    ])


def visualize_contact_3d(pcd, grasp_3d, new_grasp_pose):
    grasp_3d = transform_matrix(grasp_3d)
    new_grasp_pose = transform_matrix(new_grasp_pose)
    curr_grasp_frame = create_coordinate_frame(grasp_3d, size=0.05)
    goal_grasp_frame = create_coordinate_frame(new_grasp_pose, size=0.05)

    print("New Grasp Point :\n", new_grasp_pose)

    o3d.visualization.draw_geometries([pcd, curr_grasp_frame, goal_grasp_frame], window_name="Grasp Pose in 3D")

def plate_grasp_2d(raw_image, plate_mask, container_mask):
    # Centers and contours
    plate_center, plate_cnt = find_center(plate_mask)
    r = min(np.linalg.norm(np.array(p[0]) - plate_center) for p in plate_cnt)
    container_center, container_cnt = find_center(container_mask)

    # Direction vector and grasp points
    v = container_center - plate_center
    v_unit = v / np.linalg.norm(v)
    grasp_point = plate_center - r * v_unit
    nearest_point = plate_center + r * v_unit

    # Intersection
    intersections = line_intersects_contour(nearest_point.astype(np.float32),
                                            container_center.astype(np.float32),
                                            container_cnt)
    intersection_pt = None
    if intersections:
        intersection_pt = min(intersections, key=lambda pt: np.linalg.norm(pt - nearest_point)).astype(int)
    else:
        intersection_pt = np.array([-1, -1])

    # # Visualization
    # visualize_2d(
    #     raw_image, plate_center, container_center,
    #     grasp_point, nearest_point, r, intersection_pt
    # )
    return grasp_point, nearest_point, container_center, plate_center, intersection_pt

def find_contact_point(grasp_3d, entry_3d, edge_3d, container_height=0.4):
    move_vec = edge_3d - entry_3d
    new_grasp_xy = grasp_3d + move_vec
    new_grasp_pose = np.array([
        new_grasp_xy[0], new_grasp_xy[1], container_height
    ])
    return new_grasp_pose

def main():
    rgb, depth, mask1, mask2 = load_images_2(pizza_grasp_path, pizza_grasp_depth_path, pizza_grasp_plate_path, pizza_grasp_container_path)
    grasp_point, nearest_point, container_center, plate_center, intersection_pt = plate_grasp_2d(rgb, mask1, mask2)

    pcd = generate_point_cloud_2(rgb, depth, mask1, mask2, depth_K, color_K, R, t)

    # find plate grasp pose
    grasp_pose = point_mapper(grasp_point, pcd)
    entry_3d = point_mapper(nearest_point, pcd)
    edge_3d = point_mapper(intersection_pt, pcd)

    visualize_3d(pcd, grasp_pose)

    # find
    new_grasp_pose = find_contact_point(grasp_pose, entry_3d, edge_3d, container_height=0.4)
    visualize_contact_3d(pcd, grasp_pose, new_grasp_pose)

if __name__ == "__main__":
    main()