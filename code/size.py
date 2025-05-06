import cv2

# 이미지 읽기
img = cv2.imread('mask.jpg')

# shape를 통해 사이즈 확인
height, width = img.shape[:2]

print(f"Width: {width}, Height: {height}")