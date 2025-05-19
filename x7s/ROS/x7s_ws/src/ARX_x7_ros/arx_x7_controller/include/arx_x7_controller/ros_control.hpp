// ros_control.hpp
#pragma once

#include <ros/package.h>
#include "ros/ros.h"
#include "arx_x7_controller/Pose3D.h"
#include "arx_x7_controller/SinglePose.h"
#include "arx_x7_controller/Pose_loop.h"
#include "arx_x7_controller/ActionSuccess.h"
#include <chrono>
#include <thread>
#include <arm_control/PosCmd.h>
#include <arm_control/JointControl.h>
#include <thread>
#include <tuple>


namespace arx::x7 {

class X7StateInterface {
public:
  X7StateInterface(ros::NodeHandle& nh);

  // get current status
  void jointInfoCallbackLeft(const arm_control::JointInformation::ConstPtr& msg);
  void eePoseCallbackLeft(const arm_control::PosCmd::ConstPtr& msg);
  void jointInfoCallbackRight(const arm_control::JointInformation::ConstPtr& msg);
  void eePoseCallbackRight(const arm_control::PosCmd::ConstPtr& msg);


  std::vector<double> get_latest_ee_pose(bool is_left);
  std::vector<double> get_latest_joint_info(bool is_left);

  // End-effector pose control
  void set_ee_pose_cmd(ros::NodeHandle& nh, bool is_left, const std::vector<double>& pose_xyzrpy, double gripper);

  // Joint positions control
  void set_joint_pos_cmd(ros::NodeHandle& nh, bool is_left, const std::vector<double>& joint_angles);

  std::vector<double> single_grasp_xyz;
  std::vector<double> left_grasp_xyz;
  std::vector<double> right_grasp_xyz;
  int num;
  bool perception;
  bool single_grasp_received = false;
  bool single_moved_grasp_received = false; // task3 pizza
  bool bimanual_grasp_received = false;
  bool bimanual_close_grasp_received = false; // task3 close container
  bool single_grasp_is_left = false;
  bool bimanual_loop = false; // task3 close container
  bool action_fin = false;

  // get pose
  bool singlePoseCallback(arx_x7_controller::SinglePose::Request& req, arx_x7_controller::SinglePose::Response& res);
  bool singleMovedPoseCallback(arx_x7_controller::SinglePose::Request& req, arx_x7_controller::SinglePose::Response& res);
  bool bimanualPoseCallback(arx_x7_controller::Pose3D::Request& req, arx_x7_controller::Pose3D::Response& res);
  bool bimanualClosePoseCallback(arx_x7_controller::Pose3D::Request& req, arx_x7_controller::Pose3D::Response& res);
  bool poseLoopCallback(arx_x7_controller::Pose_loop::Request& req, arx_x7_controller::Pose_loop::Response& res);
  bool actionCallback(arx_x7_controller::ActionSuccess::Request& req, arx_x7_controller::ActionSuccess::Response& res);

  // Getter
  std::pair<bool, std::vector<double>> get_single_grasp_pose();
  std::pair<bool, std::vector<double>> get_single_moved_grasp_pose();
  std::pair<std::vector<double>, std::vector<double>> get_bimanual_grasp_pose();
  std::pair<std::vector<double>, std::vector<double>> get_bimanual_close_grasp_pose();
  std::tuple<std::vector<double>, std::vector<double>, int> get_pose_loop();
  bool is_action_fin();

private:
  ros::Subscriber joint_info_sub_left;
  ros::Subscriber ee_pose_sub_left;
  ros::Subscriber joint_info_sub_right;
  ros::Subscriber ee_pose_sub_right;

  arm_control::JointInformation latest_joint_info_left;
  arm_control::JointInformation latest_joint_info_right;
  arm_control::PosCmd latest_ee_pose_left;
  arm_control::PosCmd latest_ee_pose_right;

  ros::ServiceServer single_service;
  ros::ServiceServer single_moved_service;
  ros::ServiceServer bimanual_service;
  ros::ServiceServer bimanual_close_service;
  ros::ServiceServer pose_loop_service;
  ros::ServiceServer action_service;
};
}
