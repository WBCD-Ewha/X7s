import numpy as np
import cv2
import open3d as o3d
from util import depth_K, color_K, R, t

def point_mapper(grasp_point: np.ndarray, pcd: o3d.geometry.PointCloud) -> np.ndarray:
    """
    grasp_point (color image 기준 2D 좌표)와 가장 가까운 point cloud 상의 3D 점을 반환
    """
    points = np.asarray(pcd.points)
    projected_uvs = []

    for pt in points:
        # depth → color extrinsic
        p_color = R @ pt + t
        if p_color[2] <= 0:
            projected_uvs.append([-1, -1])
            continue

        u = int((p_color[0] * color_K['fx']) / p_color[2] + color_K['cx'])
        v = int((p_color[1] * color_K['fy']) / p_color[2] + color_K['cy'])
        projected_uvs.append([u, v])

    projected_uvs = np.array(projected_uvs)

    # 유효한 범위만 필터링
    valid_mask = (projected_uvs[:, 0] >= 0) & (projected_uvs[:, 0] < color_K['width']) & \
                 (projected_uvs[:, 1] >= 0) & (projected_uvs[:, 1] < color_K['height'])

    valid_uvs = projected_uvs[valid_mask]
    valid_points = points[valid_mask]

    # 가장 가까운 (u,v) 찾기
    dists = np.linalg.norm(valid_uvs - grasp_point, axis=1)
    min_idx = np.argmin(dists)

    return valid_points[min_idx]