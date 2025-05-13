// ros_control.hpp
#pragma once

#include <ros/package.h>
#include "ros/ros.h"
#include <chrono>
#include <thread>
#include <arm_control/PosCmd.h>
#include <arm_control/JointControl.h>
#include <thread>


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

private:
  ros::Subscriber joint_info_sub_left;
  ros::Subscriber ee_pose_sub_left;
  ros::Subscriber joint_info_sub_right;
  ros::Subscriber ee_pose_sub_right;

  arm_control::JointInformation latest_joint_info_left;
  arm_control::JointInformation latest_joint_info_right;
  arm_control::PosCmd latest_ee_pose_left;
  arm_control::PosCmd latest_ee_pose_right;
};
}
