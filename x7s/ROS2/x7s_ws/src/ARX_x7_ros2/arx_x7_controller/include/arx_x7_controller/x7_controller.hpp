#pragma once

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>

#include "arx_x7_src/interfaces/InterfacesThread.hpp"

#include "arx5_arm_msg/msg/robot_cmd.hpp"
#include "arm_control/msg/joint_control.hpp"
#include "arx5_arm_msg/msg/robot_status.hpp"
#include "arm_control/msg/pos_cmd.hpp"

namespace arx::x7 {
class X7Controller : public rclcpp::Node {
 public:
  X7Controller();
  void cleanup() {
    interfaces_ptr_.reset();
  }

  void CmdCallback(const arm_control::msg::JointControl::SharedPtr msg);
  void PubState();

  void VrCmdCallback(const arm_control::msg::PosCmd::SharedPtr msg);
  void VrPubState();

 private:
  std::shared_ptr<InterfacesThread> interfaces_ptr_;

  // 通常 & remote从机模式下
  rclcpp::Publisher<arx5_arm_msg::msg::RobotStatus>::SharedPtr joint_state_publisher_;
  // vr模式下
  rclcpp::Publisher<arm_control::msg::PosCmd>::SharedPtr vr_joint_state_publisher_;

  // 通常模式下
  rclcpp::Subscription<arm_control::msg::JointControl>::SharedPtr joint_state_subscriber_;
  // vr模式下
  rclcpp::Subscription<arm_control::msg::PosCmd>::SharedPtr vr_joint_state_subscriber_;

  rclcpp::TimerBase::SharedPtr timer_;
};
}