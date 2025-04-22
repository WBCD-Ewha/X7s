import json

# 1. 원본 JSON 경로
input_path = "camera_intrinsics.json"
output_path = "camera.json"

# 2. JSON 읽기
with open(input_path, "r") as f:
    intrinsics = json.load(f)

# 3. cam_K 구성
cam_K = [
    intrinsics["focal_lengths_in_pixels"]["fx"], 0.0, intrinsics["principal_point_in_pixels"]["cx"],
    0.0, intrinsics["focal_lengths_in_pixels"]["fy"], intrinsics["principal_point_in_pixels"]["cy"],
    0.0, 0.0, 1.0
]

# 4. 새로운 JSON 포맷
converted = {
    "cam_K": cam_K,
    "depth_scale": 1.0
}

# 5. 저장
with open(output_path, "w") as f:
    json.dump(converted, f, indent=4)

print(f"Finish: {output_path}")