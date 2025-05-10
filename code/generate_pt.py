import open3d as o3d
import numpy as np
import cv2
import argparse

def load_images(rgb_path, depth_path, mask_path):
    rgb = cv2.imread(rgb_path)
    depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)  # uint16
    mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)    # mask는 grayscale (0:배경, 255:객체)
    return rgb, depth, mask

def load_images_2(rgb_path, depth_path, mask_path1, mask_path2):
    rgb = cv2.imread(rgb_path)
    depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)  # uint16
    mask1 = cv2.imread(mask_path1, cv2.IMREAD_GRAYSCALE)
    mask2 = cv2.imread(mask_path2, cv2.IMREAD_GRAYSCALE)
    return rgb, depth, mask1, mask2


def generate_point_cloud(rgb, depth, mask, depth_intrinsic, color_intrinsic, R, t, depth_scale=1000.0):
    points = []
    colors = []

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

            # Depth -> Color 변환
            p_color = R @ p_depth + t

            u_color = int((p_color[0] * color_intrinsic['fx']) / p_color[2] + color_intrinsic['cx'])
            v_color = int((p_color[1] * color_intrinsic['fy']) / p_color[2] + color_intrinsic['cy'])

            if 0 <= u_color < color_intrinsic['width'] and 0 <= v_color < color_intrinsic['height']:
                if mask[v_color, u_color] == 0 :
                    continue

                color = rgb[v_color, u_color, :] / 255.0
                points.append(p_depth)
                colors.append(color)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(points))
    pcd.colors = o3d.utility.Vector3dVector(np.array(colors))
    return pcd

def generate_point_cloud_2(
    rgb, depth, mask1, mask2,
    depth_intrinsic, color_intrinsic,
    R, t, depth_scale=1000.0
):
    points = []
    colors = []

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

            # Depth → Color extrinsic 적용
            p_color = R @ p_depth + t

            u_color = int((p_color[0] * color_intrinsic['fx']) / p_color[2] + color_intrinsic['cx'])
            v_color = int((p_color[1] * color_intrinsic['fy']) / p_color[2] + color_intrinsic['cy'])

            if 0 <= u_color < color_intrinsic['width'] and 0 <= v_color < color_intrinsic['height']:
                if mask1[v_color, u_color] == 0 and mask2[v_color, u_color] == 0:
                    continue

                color = rgb[v_color, u_color, :] / 255.0
                points.append(p_depth)
                colors.append(color)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(points))
    pcd.colors = o3d.utility.Vector3dVector(np.array(colors))
    return pcd


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