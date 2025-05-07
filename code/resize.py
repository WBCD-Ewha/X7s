import cv2

# 1. 깨진 큰 mask.jpg 읽기
mask = cv2.imread('mask.jpg', cv2.IMREAD_UNCHANGED)

# 2. 원하는 정상 크기
target_width = 640
target_height = 480

# 3. resize
mask_resized = cv2.resize(mask, (target_width, target_height), interpolation=cv2.INTER_NEAREST)

# 4. 다시 저장
cv2.imwrite('mask_fixed.jpg', mask_resized)

print("저장 완료: mask_fixed.jpg (640x480)")