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


void move_arm(X7StateInterface& controller, ros::NodeHandle& nh,
              const Eigen::Matrix4d& left_grasp_pose,
              const Eigen::Matrix4d& right_grasp_pose,
              const Eigen::Matrix4d& left_goal_pose,
              const Eigen::Matrix4d& right_goal_pose,
              const std::vector<double>& left_quat_rpy,
              const std::vector<double>& right_quat_rpy,
              const Eigen::Matrix4d& camera_extrinsic,
              double press_offset) {


  move_to_start_pose(controller, nh);
  // grasp pose transformation w.r.t world frame
  Eigen::Matrix4d left_grasp_world = transform_camera_to_world(left_grasp_pose, camera_extrinsic);
  Eigen::Matrix4d right_grasp_world = transform_camera_to_world(right_grasp_pose, camera_extrinsic);

  Eigen::Matrix4d left_goal_world = transform_camera_to_world(left_goal_pose, camera_extrinsic);
  Eigen::Matrix4d right_goal_world = transform_camera_to_world(right_goal_pose, camera_extrinsic);


  // matrix conversion to xyzrpy
  std::vector<double> left_grasp_xyz, right_grasp_xyz, left_grasp_rpy, right_grasp_rpy;
  std::vector<double> left_goal_xyz, right_goal_xyz, left_goal_rpy, right_goal_rpy;
  mat_to_xyzrpy(left_grasp_world, left_grasp_xyz, left_grasp_rpy);
  mat_to_xyzrpy(right_grasp_world, right_grasp_xyz, right_grasp_rpy);
  mat_to_xyzrpy(left_goal_world, left_goal_xyz, left_goal_rpy);
  mat_to_xyzrpy(right_goal_world, right_goal_xyz, right_goal_rpy);

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

  // move to gaol pose
  std::vector<double> left_goal = {left_goal_xyz[0], left_goal_xyz[1], left_goal_xyz[2],left_goal_rpy[0], left_goal_rpy[1], left_goal_rpy[2]};
  std::vector<double> right_goal = {right_goal_xyz[0], right_goal_xyz[1], right_goal_xyz[2], right_goal_rpy[0], right_goal_rpy[1], right_goal_rpy[2]};
  std::thread left_goal_thread(&X7StateInterface::set_ee_pose_cmd, &controller, std::ref(nh), true, left_grasp, 0.0);
  std::thread right_goal_thread(&X7StateInterface::set_ee_pose_cmd, &controller, std::ref(nh), false, right_grasp, 0.0);
  left_goal_thread.join();
  right_goal_thread.join();
  sleep_sec(1.0);

  // push the lid
  Eigen::Matrix3d left_rot = left_goal_world.block<3,3>(0,0);
  Eigen::Matrix3d right_rot = right_goal_world.block<3,3>(0,0);

  Eigen::Vector3d z_axis_left = left_rot.col(2);    // Z축
  Eigen::Vector3d z_axis_right = right_rot.col(2);  // Z축

  Eigen::Vector3d pos_left = left_goal_world.block<3,1>(0,3);
  Eigen::Vector3d pos_right = right_goal_world.block<3,1>(0,3);

  Eigen::Vector3d push_left = pos_left - press_offset * z_axis_left;
  Eigen::Vector3d push_right = pos_right - press_offset * z_axis_right;

  std::vector<double> left_push = {
    push_left.x(), push_left.y(), push_left.z(),
    left_goal_rpy[0], left_goal_rpy[1], left_goal_rpy[2]
  };

  std::vector<double> right_push = {
    push_right.x(), push_right.y(), push_right.z(),
     right_goal_rpy[0], right_goal_rpy[1],  right_goal_rpy[2]
  };

  std::thread left_push_thread(&X7StateInterface::set_ee_pose_cmd, &controller, std::ref(nh), true, left_push, 0.0);
  std::thread right_push_thread(&X7StateInterface::set_ee_pose_cmd, &controller, std::ref(nh), false, right_push, 0.0);

  left_push_thread.join();
  right_push_thread.join();

  sleep_sec(2.0);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "close_lid_node");
  ros::NodeHandle nh;

  arx::x7::X7StateInterface controller(nh);

  // 1. grasp pose
  Eigen::Matrix4d left_grasp_pose = Eigen::Matrix4d::Identity();
  left_grasp_pose(0, 3) = 0.05;  // x
  left_grasp_pose(1, 3) = 0.10;  // y
  left_grasp_pose(2, 3) = 0.01;  // z

  Eigen::Matrix4d right_grasp_pose = Eigen::Matrix4d::Identity();
  right_grasp_pose(0, 3) = 0.05;  // x
  right_grasp_pose(1, 3) = 0.00;  // y
  right_grasp_pose(2, 3) = 0.01;  // z

  // 2. goal pose
  Eigen::Matrix4d left_goal_pose = Eigen::Matrix4d::Identity();;
  left_goal_pose(0, 3) = 0.05;  // x
  left_goal_pose(1, 3) = 0.0;  // y
  left_goal_pose(2, 3) = 0.15;  // z

  Eigen::Matrix4d right_goal_pose = Eigen::Matrix4d::Identity();;
  right_grasp_pose(0, 3) = 0.05;  // x
  right_grasp_pose(1, 3) = -0.1;  // y
  right_grasp_pose(2, 3) = 0.15;  // z

  // 3. Example rpy orientation (approach)
  // TODO : check rpy
  std::vector<double> left_quat_rpy = {1.5707963,  0, -1.5707963};
  std::vector<double> right_quat_rpy = {1.5707963,  0, -1.5707963};

  // 4. extrinsic
  Eigen::Matrix4d camera_extrinsic = Eigen::Matrix4d::Identity();  // 실제 환경에 맞게 교체

  double press_offset = 0.05;

  // processing
  move_arm(controller, nh, left_grasp_pose, left_goal_pose, right_grasp_pose, right_goal_pose, left_quat_rpy, right_quat_rpy, camera_extrinsic, press_offset);


  return 0;
}
