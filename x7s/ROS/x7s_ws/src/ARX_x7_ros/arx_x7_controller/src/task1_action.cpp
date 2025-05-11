#include "arx_x7_controller/x7_controller.hpp"
#include <ros/package.h>

#include "ros/ros.h"
#include <chrono>
#include <thread>
#include <arm_control/PosCmd.h>
#include <arm_control/JointControl.h>

// End-effector pose control
void set_ee_pose_cmd(ros::NodeHandle& nh, bool is_left, const std::vector<double>& pose_xyzrpy, double gripper) {
  std::string topic_name = is_left ? "/ARX_VR_L" : "/ARX_VR_R";
  ros::Publisher pub = nh.advertise<arm_control::PosCmd>(topic_name, 10);
  ros::Duration(1.0).sleep();

  arm_control::PosCmd cmd;
  cmd.x = pose_xyzrpy[0];
  cmd.y = pose_xyzrpy[1];
  cmd.z = pose_xyzrpy[2];
  cmd.roll = pose_xyzrpy[3];
  cmd.pitch = pose_xyzrpy[4];
  cmd.yaw = pose_xyzrpy[5];
  cmd.gripper = gripper;

  ros::Rate rate(10);
  for (int i = 0; i < 20; ++i) {
    pub.publish(cmd);
    rate.sleep();
  }
}

// Joint positions control
void set_joint_pos_cmd(ros::NodeHandle& nh, bool is_left, const std::vector<double>& joint_angles) {
  std::string topic_name = is_left ? "/joint_control" : "/joint_control2";
  ros::Publisher pub = nh.advertise<arm_control::JointControl>(topic_name, 10);
  ros::Duration(1.0).sleep();

  arm_control::JointControl cmd;
  cmd.joint_pos = joint_angles;

  ros::Rate rate(10);
  for (int i = 0; i < 20; ++i) {
    pub.publish(cmd);
    rate.sleep();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "unified_commander");
  ros::NodeHandle nh;

  bool use_vr = true;         // true: end-effector 제어, false: joint 제어
  bool is_left = true;        // true: 왼팔, false: 오른팔

  if (use_vr) {
    std::vector<double> pose = {0.4, 0.0, 0.2, 0.0, 1.57, 0.0}; // xyzrpy
    double gripper = 1.0;
    set_ee_pose_cmd(nh, is_left, pose, gripper);
  } else {
    std::vector<double> joints = {0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}; // 7 joints + gripper
    set_joint_pos_cmd(nh, is_left, joints);
  }

  return 0;
}
