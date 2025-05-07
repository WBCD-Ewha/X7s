import cv2
import numpy as np

rgb_path = '../datasets/sample_0/subtask2/camera_l/image_left.png'
lid_path = '../../Grounded-Segment-Anything/outputs/container_lid.jpg'
container_path = '../../Grounded-Segment-Anything/outputs/container_2.jpg'

# 1. Load images
lid_mask = cv2.imread(lid_path, cv2.IMREAD_GRAYSCALE)
container_mask = cv2.imread(container_path, cv2.IMREAD_GRAYSCALE)
raw_image = cv2.imread(rgb_path)

# 2. container center
contours_container, _ = cv2.findContours(container_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
container_cnt = max(contours_container, key=cv2.contourArea)
M_container = cv2.moments(container_cnt)
cx_cont = int(M_container["m10"] / M_container["m00"])
cy_cont = int(M_container["m01"] / M_container["m00"])
container_center = np.array([cx_cont, cy_cont])

# 3. lid center
contours_lid, _ = cv2.findContours(lid_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
lid_cnt = max(contours_lid, key=cv2.contourArea)
M_lid = cv2.moments(lid_cnt)
cx_lid = int(M_lid["m10"] / M_lid["m00"])
cy_lid = int(M_lid["m01"] / M_lid["m00"])
lid_center = np.array([cx_lid, cy_lid])

# 5. Direction vector
v = container_center - lid_center
v_unit = v / np.linalg.norm(v)


vis = raw_image.copy()

np.save("container2_center.npy", container_center)
np.save("lid_center.npy", lid_center)

# 7. Draw on raw image
cv2.circle(vis, tuple(lid_center), 5, (0, 0, 255), -1)               # 접시 중심
cv2.circle(vis, tuple(container_center), 5, (0, 255, 0), -1)            # 컨테이너 중심

cv2.arrowedLine(vis, tuple(lid_center.astype(int)), tuple(container_center.astype(int)),
                (0, 255, 255), 2, tipLength=0.2)

cv2.imshow("Planning", vis)
cv2.waitKey(0)
cv2.destroyAllWindows()