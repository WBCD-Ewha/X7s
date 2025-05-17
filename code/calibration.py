import numpy as np

def calibrate(head_yaw, head_pitch, cam_pos):
    left_to_head_fixed = np.array([[ 9.99997121e-01, -5.07022400e-09, -2.39955934e-03, -2.11058635e-01],
     [ 1.47546327e-09,  1.00000000e+00, -1.49809381e-06, -1.50999461e-01],
     [ 2.39955934e-03,  1.49808595e-06,  9.99997121e-01,  3.42915784e-01],
     [ 0.00000000e+00, 0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]
    )

    right_to_head_fixed = np.array([[ 9.99997121e-01,  5.07022347e-09, -2.39955934e-03, -2.11058635e-01],
     [-1.47546438e-09,  1.00000000e+00,  1.49809312e-06,  1.50999469e-01],
     [ 2.39955934e-03, -1.49808527e-06,  9.99997121e-01,  3.42915784e-01],
     [ 0.00000000e+00, 0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

    #head_yaw, head_pitch 값은 함수에 파라미터로 주어짐. 단위 라디안.

    # Yaw 회전 행렬 (Z축 회전), position은 그대로
    cy, sy = np.cos(head_yaw), np.sin(head_yaw)
    head_fixed_to_head_yaw = np.array([
        [cy, -sy, 0.0, 0.0],
        [sy, cy, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ])

    # Pitch 회전 행렬 (Y축 회전), position은 z축으로 0.048 올라감 (rotation 먼저 그다음에 position 올라감)
    cp, sp = np.cos(head_pitch), np.sin(head_pitch)
    R_pitch = np.array([
        [cp, 0.0, sp],
        [0.0, 1.0, 0.0],
        [-sp, 0.0, cp]
    ])
    T_translate = np.eye(4)
    T_translate[:3, 3] = [0.0, 0.0, 0.04899967]
    T_rotate = np.eye(4)
    T_rotate[:3, :3] = R_pitch
    head_yaw_to_head_pitch = T_rotate @ T_translate

    # head_pitch → camera, orientation 고정, position만 cam_pos로 대체
    T_translate = np.eye(4)
    T_translate[:3, 3] = cam_pos

    R_cam = np.array([
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0]
    ])
    T_rotate = np.eye(4)
    T_rotate[:3, :3] = R_cam
    head_pitch_to_cam = T_rotate @ T_translate

    left_to_cam = left_to_head_fixed @ head_fixed_to_head_yaw @ head_yaw_to_head_pitch @ head_pitch_to_cam
    right_to_cam = right_to_head_fixed @ head_fixed_to_head_yaw @ head_yaw_to_head_pitch @ head_pitch_to_cam

    print("camera frame w.r.t left world:", left_to_cam)
    print("camera frame w.r.t right world:", right_to_cam)

    return left_to_cam, right_to_cam


if __name__ == "__main__":
    calibrate(head_yaw=0.0, head_pitch=0.0, cam_pos=np.array([0.0, 0.0, 0.0]))