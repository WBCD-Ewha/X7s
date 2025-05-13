#include "arx_x7_controller/x7_controller.hpp"
#include "arx_x7_controller/ros_control.hpp"
#include <ros/package.h>
#include "ros/ros.h"
#include <arm_control/PosCmd.h>
#include <arm_control/JointControl.h>
#include <Eigen/Dense>
#include <thread>
#include <Eigen/Dense>
#include <iostream>
#include <chrono>
#include <vector>
#include <cmath>

using namespace arx::x7;

void sleep_sec(double t) {
  std::this_thread::sleep_for(std::chrono::duration<double>(t));
}

Eigen::Matrix4d transform_camera_to_world(const Eigen::Matrix4d& cam_pose, const Eigen::Matrix4d& camera_extrinsic) {
    return camera_extrinsic.inverse() * cam_pose;
}

void move_to_start_pose(X7StateInterface& controller, ros::NodeHandle& nh) {
    std::vector<double> start_config = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0};
    std::thread left_thread(&X7StateInterface::set_joint_pos_cmd, &controller, std::ref(nh), true, start_config);
    std::thread right_thread(&X7StateInterface::set_joint_pos_cmd, &controller, std::ref(nh), false, start_config);
    left_thread.join();
    right_thread.join();
}

// 4x4 matrix -> xyzrpy
void mat_to_xyzrpy(const Eigen::Matrix4d& pose, std::vector<double>& xyz, std::vector<double>& rpy) {
    xyz = {pose(0, 3), pose(1, 3), pose(2, 3)};
    Eigen::Matrix3d rot = pose.block<3, 3>(0, 0);
    Eigen::Vector3d euler = rot.eulerAngles(2, 1, 0);  // ZYX
    rpy = {euler[2], euler[1], euler[0]};
}

void open_grippers(X7StateInterface& controller, ros::NodeHandle& nh) {
    /// 각 arm의 현재 pose 가져올 것 (x, y, z, r, p, y 형태)
    std::vector<double> current_left_pose = controller.get_latest_ee_pose(true);
    std::vector<double> current_right_pose = controller.get_latest_ee_pose(false);

    std::thread left_thread(&X7StateInterface::set_ee_pose_cmd, &controller, std::ref(nh), true, current_left_pose, 3.0);
    std::thread right_thread(&X7StateInterface::set_ee_pose_cmd, &controller, std::ref(nh), false, current_right_pose, 3.0);
    left_thread.join();
    right_thread.join();
}

void close_grippers(X7StateInterface& controller, ros::NodeHandle& nh) {
    /// 각 arm의 현재 pose 가져올 것 (x, y, z, r, p, y 형태)
    std::vector<double> current_left_pose = controller.get_latest_ee_pose(true);
    std::vector<double> current_right_pose = controller.get_latest_ee_pose(false);

    std::thread left_thread(&X7StateInterface::set_ee_pose_cmd, &controller, std::ref(nh), true, current_left_pose, 0.0);
    std::thread right_thread(&X7StateInterface::set_ee_pose_cmd, &controller, std::ref(nh), false, current_right_pose, 0.0);
    left_thread.join();
    right_thread.join();
}

void open_container(X7StateInterface& controller, ros::NodeHandle& nh,
              const Eigen::Matrix4d& left_grasp_pose,
              const Eigen::Matrix4d& right_grasp_pose,
              const std::vector<double>& left_quat_rpy,
              const std::vector<double>& right_quat_rpy,
              const Eigen::Matrix4d& camera_extrinsic,
              double angle_rad) {

  move_to_start_pose(controller, nh);

  // grasp pose transformation w.r.t world frame
  Eigen::Matrix4d left_grasp_world = transform_camera_to_world(left_grasp_pose, camera_extrinsic);
  Eigen::Matrix4d right_grasp_world = transform_camera_to_world(right_grasp_pose, camera_extrinsic);

  // matrix conversion to xyzrpy
  std::vector<double> left_grasp_xyz, right_grasp_xyz, left_grasp_rpy, right_grasp_rpy;
  mat_to_xyzrpy(left_grasp_world, left_grasp_xyz, left_grasp_rpy);
  mat_to_xyzrpy(right_grasp_world, right_grasp_xyz, right_grasp_rpy);

  // move to grasp pose
  std::vector<double> left_grasp = {left_grasp_xyz[0], left_grasp_xyz[1], left_grasp_xyz[2],left_grasp_rpy[0], left_grasp_rpy[1], left_grasp_rpy[2]};
  std::vector<double> right_grasp = {right_grasp_xyz[0], right_grasp_xyz[1], right_grasp_xyz[2], right_grasp_rpy[0], right_grasp_rpy[1], right_grasp_rpy[2]};
  std::thread left_thread(&X7StateInterface::set_ee_pose_cmd, &controller, std::ref(nh), true, left_grasp, 3.0);
  std::thread right_thread(&X7StateInterface::set_ee_pose_cmd, &controller, std::ref(nh), false, right_grasp, 3.0);
  left_thread.join();
  right_thread.join();
  sleep_sec(1.0);

  // close gipper
  close_grippers(controller, nh);
  sleep_sec(1.0);

  // rotate the lid flap
  Eigen::Matrix3d left_rot = left_grasp_world.block<3,3>(0,0);
  Eigen::Matrix3d right_rot = right_grasp_world.block<3,3>(0,0);

  Eigen::AngleAxisd left_pitch_rot(+angle_rad, left_rot.col(1));   // left +angle
  Eigen::AngleAxisd right_pitch_rot(-angle_rad, right_rot.col(1)); // right -angle

  Eigen::Matrix3d left_rotated = left_pitch_rot.toRotationMatrix() * left_rot;
  Eigen::Matrix3d right_rotated = right_pitch_rot.toRotationMatrix() * right_rot;

  Eigen::Vector3d left_rpy = left_rotated.eulerAngles(2, 1, 0);   // ZYX → RPY
  Eigen::Vector3d right_rpy = right_rotated.eulerAngles(2, 1, 0); // ZYX → RPY

  std::vector<double> left_rotated_pose = {
    left_grasp_xyz[0], left_grasp_xyz[1], left_grasp_xyz[2],
    left_rpy[2], left_rpy[1], left_rpy[0]
  };

  std::vector<double> right_rotated_pose = {
    right_grasp_xyz[0], right_grasp_xyz[1], right_grasp_xyz[2],
    right_rpy[2], right_rpy[1], right_rpy[0]
  };

  std::thread left_rotate_thread(&X7StateInterface::set_ee_pose_cmd, &controller, std::ref(nh), true, left_rotated_pose, 0.0);
  std::thread right_rotate_thread(&X7StateInterface::set_ee_pose_cmd, &controller, std::ref(nh), false, right_rotated_pose, 0.0);

  left_rotate_thread.join();
  right_rotate_thread.join();
  sleep_sec(2.0);

  move_to_start_pose(controller, nh);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "close_lid_node");
  ros::NodeHandle nh;

  arx::x7::X7StateInterface controller(nh);

  // 1. grasp pose
  Eigen::Matrix4d left_grasp_pose = Eigen::Matrix4d::Identity();
  left_grasp_pose(0, 3) = 0.05;  // x
  left_grasp_pose(1, 3) = 0.05;  // y
  left_grasp_pose(2, 3) = 0.01;  // z

  Eigen::Matrix4d right_grasp_pose = Eigen::Matrix4d::Identity();
  right_grasp_pose(0, 3) = 0.05;  // x
  right_grasp_pose(1, 3) = -0.05;  // y
  right_grasp_pose(2, 3) = 0.01;  // z


  // 3. Example rpy orientation (approach)
  // TODO : check rpy
  std::vector<double> left_quat_rpy = {1.0,  0, -1.5707963};
  std::vector<double> right_quat_rpy = {-1.0,  0, -1.5707963};

  // 6. rotate angle
  double angle_deg = 40.0;
  double angle_rad = angle_deg * M_PI / 180.0;

  // 4. extrinsic
  Eigen::Matrix4d camera_extrinsic = Eigen::Matrix4d::Identity();

  // processing
  open_container(controller, nh, left_grasp_pose, right_grasp_pose, left_quat_rpy, right_quat_rpy, camera_extrinsic, angle_rad);


  return 0;
}
