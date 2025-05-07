# for arm
from x7s.py.ARX_x7_python.bimanual import SingleArm
from x7s.py.ARX_x7_python.bimanual import BimanualArm
from typing import Dict, Any
import numpy as np
from scipy.spatial.transform import Rotation as R
import time


def grasp_cloth(left_arm: SingleArm, right_arm: SingleArm, grasp_poses, fixed_approach_quat, camera_extrinsic, pick_up_height=0.1):
    # TODO: 적절한 start pose로 먼저 움직이기
    left_start_config = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    right_start_config = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    left_arm.set_joint_positions(positions=left_start_config)
    right_arm.set_joint_positions(positions=right_start_config)

    left_pose_cam = grasp_poses[0]
    right_pose_cam = grasp_poses[1]
    print("left & right grasp pose in world frame: ", left_pose_cam, right_pose_cam, "\n")

    # TODO: camera frame -> world frame transformation
    # camera_extrinsic은 4x4 homogeneuos matrix, camera frame w.r.t world frame
    cam_T_world = np.linalg.inv(camera_extrinsic)
    left_pose_world = cam_T_world @ left_pose_cam
    right_pose_world = cam_T_world @ right_pose_cam
    print("left & right grasp pose in world frame: ", left_pose_world, right_pose_world, "\n")

    # pose should be pos: Desired position [x, y, z]. Shape: (3,) ori: Desired orientation (quaternion). Shape: (4,) (w, x, y, z)
    def decompose_pose(pose_4x4):
        position = pose_4x4[:3, 3]
        rot_mat = pose_4x4[:3, :3]
        quat_xyzw = R.from_matrix(rot_mat).as_quat()  # (x, y, z, w)
        quat_wxyz = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])
        return position, quat_wxyz

    left_pos, left_quat = decompose_pose(left_pose_world)
    right_pos, right_quat = decompose_pose(right_pose_world)

    # open grippers
    # TODO: check pos values
    left_arm.set_catch_pos(pos=0.2)
    right_arm.set_catch_pos(pos=0.2)

    # move to grasp pose
    # both should be np.array
    # TODO: check pose orientation
    left_success = left_arm.set_ee_pose(pos=left_pos, quat=fixed_approach_quat[0])
    right_success = right_arm.set_ee_pose(pos=right_pos, quat=fixed_approach_quat[1])
    print("grasp the cloth")
    print("left_success: ", left_success)
    print("right_success: ", right_success)

    # close grippers
    # TODO: check pos values
    left_arm.set_catch_pos(pos=0.0)
    right_arm.set_catch_pos(pos=0.0)

    # pick up the cloth
    # TODO: check pose orientation
    left_pos[2] -= pick_up_height
    right_pos[2] -= pick_up_height
    left_success = left_arm.set_ee_pose(pos=left_pos, quat=fixed_approach_quat[0])
    right_success = right_arm.set_ee_pose(pos=right_pos, quat=fixed_approach_quat[1])
    print("pick up the cloth")
    print("left_success: ", left_success)
    print("right_success: ", right_success)


def stretch_cloth(left_arm: SingleArm, right_arm: SingleArm, max_torque_threshold=1.2, step_size=0.02):

    while True:
        # get current pose/torques
        left_ee_pose = left_arm.get_ee_pose()
        right_ee_pose = right_arm.get_ee_pose()
        print("left_ee_pose: ", left_ee_pose)
        print("right_ee_pose: ", right_ee_pose)

        left_joint_curr = left_arm.get_joint_currents()
        right_joint_curr = right_arm.get_joint_currents()
        print("left_joint_currents: ", left_joint_curr)
        print("right_joint_currents: ", right_joint_curr)

        # stop if torque threshold reached
        # TODO: check force threshold
        if np.any(left_joint_curr > max_torque_threshold) or np.any(right_joint_curr > max_torque_threshold):
            print("Force threshold reached. Stopping stretch.")
            break

        # stretch the cloth (move both arms simulat)
        # TODO: check world frame axis directions / step size / simultaneous
        left_ee_pose[1] -= step_size
        right_ee_pose[1] += step_size
        success_left = left_arm.set_ee_pose(pos=left_ee_pose[:3], quat=left_ee_pose[3:])
        success_right = right_arm.set_ee_pose(pos=right_ee_pose[:3], quat=right_ee_pose[3:])

        if not (success_left and success_right):
            print("Pose set failed. Exiting.")
            break

        time.sleep(0.1)


def fling_cloth(left_arm: SingleArm, right_arm: SingleArm, left_fling_pos, right_fling_pos):
    # get current pose/torques
    left_pos = left_arm.get_joint_positions()
    right_pos = right_arm.get_joint_positions()
    print("left_ee_pose: ", left_pos)
    print("right_ee_pose: ", right_pos)

    # move up (velocity control) (아예 다 핀다)
    # TODO: check joint configuration / simultaneous
    success_left = left_arm.set_joint_positions(positions=left_fling_pos, joint_names = ["joint5", "joint8", "joint10"])
    success_right = right_arm.set_joint_positions(positions=right_fling_pos,joint_names = ["joint14", "joint17", "joint19"])
    print("fling the cloth")
    print("left_success: ", success_left)
    print("right_success: ", success_right)

    # place cloth
    success_left = left_arm.set_joint_positions(positions=left_pos)
    success_right = right_arm.set_joint_positions(positions=right_pos)
    print("place the cloth")
    print("left_success: ", success_left)
    print("right_success: ", success_right)


if __name__ == "__main__":
    arm_config_0: Dict[str, Any] = {
        "can_port": "can1",
        "type": 0,
        # Add necessary configuration parameters for the left arm
    }

    arm_config_1: Dict[str, Any] = {
        "can_port": "can3",
        "type": 0,
        # Add necessary configuration parameters for the left arm
    }

    # TODO: which is left/right
    left_arm = SingleArm(arm_config_0)
    right_arm = SingleArm(arm_config_1)


    grasp_poses_cam = np.array([[0.18389242,0.20552573 , 0.47349079, 1, 0, 0, 0], [0.32238394,-0.09454172 , 0.60695245, 1, 0, 0, 0]])
    fixed_approach_quat = np.array([[1.0, 0.0, 1.0, 0.0], [1.0, 0.0, 1.0, 0.0]]) # TODO: -1, 1 넣어서 다 해보기
    grasp_cloth(left_arm=left_arm, right_arm=right_arm, grasp_poses=grasp_poses_cam, fixed_approach_quat=fixed_approach_quat, camera_extrinsic=extrinsic, pick_up_height=0.1)

    stretch_cloth(left_arm=left_arm, right_arm=right_arm, max_torque_threshold=1.2, step_size=0.02)

    # joint5, joint8, joint10 / joint14, joint17, joint19
    fling_cloth(left_arm=left_arm, right_arm=right_arm, left_fling_pos=[-1.57, -1.57, 0.0], right_fling_pos=[1.57, 1.57, 0.0])