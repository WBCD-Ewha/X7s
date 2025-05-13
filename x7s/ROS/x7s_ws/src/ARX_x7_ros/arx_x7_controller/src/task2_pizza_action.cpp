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

void mat_to_xyzrpy(const Eigen::Matrix4d& pose, std::vector<double>& xyz, std::vector<double>& rpy) {
    xyz = {pose(0, 3), pose(1, 3), pose(2, 3)};
    Eigen::Matrix3d rot = pose.block<3, 3>(0, 0);
    Eigen::Vector3d euler = rot.eulerAngles(2, 1, 0);  // ZYX
    rpy = {euler[2], euler[1], euler[0]};
}

void open_grippers(X7StateInterface& controller, ros::NodeHandle& nh, bool is_left) {
    if (is_left) {
        std::vector<double> current_left_pose = controller.get_latest_ee_pose(true);
        controller.set_ee_pose_cmd(nh, true, current_left_pose, 3.0);
    } else {
        std::vector<double> current_right_pose = controller.get_latest_ee_pose(false);
        controller.set_ee_pose_cmd(nh, false, current_right_pose, 3.0);
    }
}

void close_grippers(X7StateInterface& controller, ros::NodeHandle& nh, bool is_left) {
    if (is_left) {
        std::vector<double> current_left_pose = controller.get_latest_ee_pose(true);
        controller.set_ee_pose_cmd(nh, true, current_left_pose, 0.0);
    } else {
        std::vector<double> current_right_pose = controller.get_latest_ee_pose(false);
        controller.set_ee_pose_cmd(nh, false, current_right_pose, 0.0);
    }
}

// camera frame -> world frame transformation
Eigen::Matrix4d transform_camera_to_world(const Eigen::Matrix4d& cam_pose, const Eigen::Matrix4d& camera_extrinsic) {
    return camera_extrinsic.inverse() * cam_pose;
}

void grasp_and_place(X7StateInterface& controller, ros::NodeHandle& nh, bool is_left,
                     const Eigen::Matrix4d& object_pose_cam,
                     const Eigen::Matrix4d& goal_pose_cam,
                     const Eigen::Matrix4d& camera_extrinsic,
                     const std::vector<double>& grasp_quat_rpy,
                     const std::vector<double>& goal_quat_rpy,
                     double angle_rad) {

  // 1. cam to world
  Eigen::Matrix4d object_pose_world = transform_camera_to_world(object_pose_cam, camera_extrinsic);
  Eigen::Matrix4d goal_pose_world = transform_camera_to_world(goal_pose_cam, camera_extrinsic);

   // matrix conversion to xyzrpy
   std::vector<double> plate_xyz, plate_rpy, goal_xyz, goal_rpy;
   mat_to_xyzrpy(object_pose_world, plate_xyz, plate_rpy);
   mat_to_xyzrpy(goal_pose_world, goal_xyz, goal_rpy);

   std::vector<double> grasp_plate = {plate_xyz[0], plate_xyz[1], plate_xyz[2], plate_rpy[0], plate_rpy[1], plate_rpy[2]};
   std::vector<double> grasp_goal = {goal_xyz[0], goal_xyz[1], goal_xyz[2], goal_rpy[0], goal_rpy[1], goal_rpy[2]};

  // TODO: check orientation
  // 1. grasp the plate
  open_grippers(controller, nh, is_left);
  controller.set_ee_pose_cmd(nh, is_left, grasp_plate, 0.0);

  sleep_sec(1.0);

  close_grippers(controller, nh, is_left);

  // 2. move to goal pose
  controller.set_ee_pose_cmd(nh, is_left,grasp_goal, 0.0);
  sleep_sec(1.0);

  // 7. Goal pose - pitch
  // TODO : check orientation
  Eigen::Matrix3d rot = goal_pose_world.block<3,3>(0,0);
  Eigen::AngleAxisd y_rot(angle_rad, rot.col(1));
  Eigen::Matrix3d rotated = y_rot.toRotationMatrix() * rot;

  Eigen::Vector3d euler = rotated.eulerAngles(0, 1, 2);  // RPY
  Eigen::Vector3d pos = goal_pose_world.block<3,1>(0,3);

  std::vector<double> rotated_goal_pose = {
    pos.x(), pos.y(), pos.z(),
    euler[0], euler[1], euler[2]
  };
  controller.set_ee_pose_cmd(nh, is_left, rotated_goal_pose, 0.0);

  // return to obj pose
  controller.set_ee_pose_cmd(nh, is_left, grasp_plate, 0.0);
  open_grippers(controller, nh, is_left);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pick_and_place_node");
  ros::NodeHandle nh;

  arx::x7::X7StateInterface controller(nh);

  // 1. grasp pose
  Eigen::Matrix4d object_pose_cam = Eigen::Matrix4d::Identity();
  object_pose_cam(0, 3) = 0.05;  // x
  object_pose_cam(1, 3) = 0.10;  // y
  object_pose_cam(2, 3) = 0.20;  // z  //TODO

  // 2. Goal pose
  Eigen::Matrix4d goal_pose_cam = Eigen::Matrix4d::Identity();
  goal_pose_cam(0, 3) = 0.05;  // x
  goal_pose_cam(1, 3) = -0.10;  // y
  goal_pose_cam(2, 3) = 0.4;  // z

  // Example rpy orientation (approach)
  // TODO :  check orientation
  std::vector<double> grasp_quat_rpy = {-1.0, 0.0, -1.0};
  std::vector<double> goal_quat_rpy = {-1.0, 0.0, -1.0};

  // 4.extrinsic
  // TODO: check Extrinsic
  Eigen::Matrix4d camera_extrinsic = Eigen::Matrix4d::Identity();

  // 5. choose arm
  Eigen::Matrix4d cam_T_world = camera_extrinsic.inverse();
  Eigen::Matrix4d object_pose_world = Eigen::Matrix4d(cam_T_world) * object_pose_cam;
  Eigen::Vector3d obj_pos = object_pose_world.block<3,1>(0,3);
  bool use_left = obj_pos.y() > 0;

  ROS_INFO_STREAM("Using " << (use_left ? "LEFT" : "RIGHT") << " arm");

  // 6. rotate angle
  double angle_deg = 80.0;
  double angle_rad = angle_deg * M_PI / 180.0;

  // 7. processing
  grasp_and_place(controller, nh, use_left, object_pose_cam, goal_pose_cam, camera_extrinsic, grasp_quat_rpy, goal_quat_rpy, use_left ? angle_rad : -angle_rad);

  return 0;
}
