import pyrealsense2 as rs

pipeline = rs.pipeline()
config = rs.config()
pipeline.start(config)

# Get device info
profile = pipeline.get_active_profile()
depth_stream = profile.get_stream(rs.stream.depth)
color_stream = profile.get_stream(rs.stream.color)

# Get extrinsics from depth to color
extrinsics = depth_stream.get_extrinsics_to(color_stream)

print(extrinsics.rotation)    # 3x3 rotation matrix
print(extrinsics.translation) # 3D translation vector