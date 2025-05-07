import cv2
import numpy as np

rgb_path = '../datasets/sample_0/subtask1/camera_h/image_left.png'
plate_path = '../../Grounded-Segment-Anything/outputs/plate_0.jpg'
container_path = '../../Grounded-Segment-Anything/outputs/container_1.jpg'

# 1. Load images
plate_mask = cv2.imread(plate_path, cv2.IMREAD_GRAYSCALE)
container_mask = cv2.imread(container_path, cv2.IMREAD_GRAYSCALE)
raw_image = cv2.imread(rgb_path)  # ← 실제 촬영된 원본 이미지

# 2. Plate center
contours_plate, _ = cv2.findContours(plate_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
plate_cnt = max(contours_plate, key=cv2.contourArea)
M_plate = cv2.moments(plate_cnt)
cx_plate = int(M_plate["m10"] / M_plate["m00"])
cy_plate = int(M_plate["m01"] / M_plate["m00"])
plate_center = np.array([cx_plate, cy_plate])

# 3. Radius
r = min(np.linalg.norm(np.array(p[0]) - plate_center) for p in plate_cnt)

# 4. Container center
contours_container, _ = cv2.findContours(container_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
container_cnt = max(contours_container, key=cv2.contourArea)
M_container = cv2.moments(container_cnt)
cx_cont = int(M_container["m10"] / M_container["m00"])
cy_cont = int(M_container["m01"] / M_container["m00"])
container_center = np.array([cx_cont, cy_cont])

# 5. Direction vector
v = container_center - plate_center
v_unit = v / np.linalg.norm(v)

# 6. Intersection points
grasp_point = plate_center - r * v_unit
entry_point = plate_center + r * v_unit

# 8. Find intersection between yellow line and container contour
def line_intersects_contour(p1, p2, contour):
    intersections = []
    for i in range(len(contour)):
        a1 = contour[i][0]
        a2 = contour[(i + 1) % len(contour)][0]
        inter_pt = compute_intersection(p1, p2, a1, a2)
        if inter_pt is not None:
            intersections.append(inter_pt)
    return intersections

def compute_intersection(p1, p2, q1, q2):
    # Line p: p1 + t(p2 - p1), Line q: q1 + s(q2 - q1)
    A = np.array([[p2[0] - p1[0], q1[0] - q2[0]],
                  [p2[1] - p1[1], q1[1] - q2[1]]])
    b = np.array([q1[0] - p1[0], q1[1] - p1[1]])

    if np.linalg.det(A) == 0:
        return None  # 평행한 경우

    t, s = np.linalg.solve(A, b)
    if 0 <= s <= 1:  # q 선분 내에 있을 때만 인정
        return p1 + t * (p2 - p1)
    return None

# 실제 교점 계산
intersections = line_intersects_contour(entry_point.astype(np.float32), container_center.astype(np.float32), container_cnt)

vis = raw_image.copy()

if intersections:
    nearest_pt = min(intersections, key=lambda pt: np.linalg.norm(pt - entry_point))
    nearest_pt = nearest_pt.astype(int)
    cv2.circle(vis, tuple(nearest_pt), 6, (0, 165, 255), -1)  # 주황색 점 표시
    print("Intersection with container:", nearest_pt)
else:
    print("No intersection with container contour found.")

np.save("grasp_point.npy", grasp_point)
np.save("entry_point.npy", entry_point)
np.save("container_center.npy", container_center)
np.save("plate_center.npy", plate_center)
np.save("intersection_point.npy", nearest_pt)

# 7. Draw on raw image
cv2.circle(vis, tuple(plate_center), int(r), (255, 0, 255), 2)         # 접시 원
cv2.circle(vis, tuple(plate_center), 5, (0, 0, 255), -1)               # 접시 중심
cv2.circle(vis, tuple(container_center), 5, (0, 255, 0), -1)            # 컨테이너 중심
cv2.circle(vis, tuple(grasp_point.astype(int)), 7, (255, 255, 0), -1)  # grasp 위치
cv2.circle(vis, tuple(entry_point.astype(int)), 7, (0, 255, 255), -1)  # entry 방향

cv2.arrowedLine(vis, tuple(grasp_point.astype(int)), tuple(container_center.astype(int)),
                (0, 255, 255), 2, tipLength=0.2)

cv2.imshow("Grasp Planning", vis)
cv2.waitKey(0)
cv2.destroyAllWindows()