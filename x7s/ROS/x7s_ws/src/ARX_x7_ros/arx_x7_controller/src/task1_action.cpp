#include "arx_x7_controller/x7_controller.hpp"
#include <ros/package.h>

#include "ros/ros.h"
#include <chrono>
#include <thread>
#include <arm_control/PosCmd.h>
#include <arm_control/JointControl.h>
#include <thread>

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
  for (size_t i = 0; i < 8 && i < joint_angles.size(); ++i) {
  cmd.joint_pos[i] = static_cast<float>(joint_angles[i]);
}

  ros::Rate rate(10);
  for (int i = 0; i < 20; ++i) {
    pub.publish(cmd);
    rate.sleep();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "task1_action");
  ros::NodeHandle nh;

  bool use_vr = true;

  if (use_vr) {
    std::vector<double> left_pose = {0.1, 0.2, 0.2, 0.0, 0.0, 0.0};
    std::vector<double> right_pose = {0.1, -0.2, 0.2, 0.0, 0.0, 0.0};
    double left_gripper = 3.0;
    double right_gripper = 3.0;

    // 왼팔, 오른팔을 각각 스레드에서 실행
    std::thread left_thread(set_ee_pose_cmd, std::ref(nh), true, left_pose, left_gripper);
    std::thread right_thread(set_ee_pose_cmd, std::ref(nh), false, right_pose, right_gripper);

    left_thread.join();
    right_thread.join();

  } else {
    std::vector<double> joints = {-0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2};
    set_joint_pos_cmd(nh, true, joints);
  }

  return 0;
}

