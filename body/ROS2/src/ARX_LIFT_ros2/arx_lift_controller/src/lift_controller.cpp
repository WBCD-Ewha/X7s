#include <rclcpp/rclcpp.hpp>
#include <arx_lift_src/lift_head_control_loop.h>
#include <arm_control/msg/pos_cmd.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <csignal>

std::shared_ptr<LiftHeadControlLoop> control_loop;
void signalHandler(int signum) {
  control_loop.reset();
  rclcpp::shutdown();
}

int main(int argc, char **argv) {
  std::signal(SIGHUP, signalHandler);
  std::signal(SIGTERM, signalHandler);
  std::signal(SIGINT, signalHandler);

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("lift_controller");
  int robot_type = node->declare_parameter("robot_type", 0);
  control_loop = std::make_shared<LiftHeadControlLoop>("can5", static_cast<LiftHeadControlLoop::RobotType>(robot_type));
  int running_state = 2;
  double lift_height = 0;
  auto sub = node->create_subscription<arm_control::msg::PosCmd>("/ARX_VR_L", 1,
                                                                 [&](const arm_control::msg::PosCmd &msg) {
                                                                   control_loop->setHeight(msg.height / 41.54);
                                                                   control_loop->setWaistPos(msg.temp_float_data[0]);
                                                                   control_loop->setHeadYaw(msg.head_yaw);
                                                                   control_loop->setHeadPitch(-msg.head_pit);
                                                                   if (robot_type == 0)
                                                                     control_loop->setChassisCmd(msg.chx * 1/2.5,
                                                                                                 -msg.chy * 1/2.5,
                                                                                                 msg.chz * 1/2.5,
                                                                                                 msg.mode1);
                                                                   else
                                                                     control_loop->setChassisCmd(msg.chx * 1/5,
                                                                                                 -msg.chy * 1/5,
                                                                                                 msg.chz * 1/5,
                                                                                                 msg.mode1);

                                                                 });
  rclcpp::Time last_callback_time = rclcpp::Clock().now();
  auto joy_sub = node->create_subscription<sensor_msgs::msg::Joy>("/joy", 1, [&](const sensor_msgs::msg::Joy &msg) {
    double duration = (rclcpp::Clock().now() - last_callback_time).seconds();
    if (duration > 1)
      duration = 0;
    lift_height +=
        msg.axes[1] * (rclcpp::Clock().now() - last_callback_time).seconds();
    if (lift_height > 0.48)
      lift_height = 0.48;
    else if (lift_height < 0.)
      lift_height = 0.;
    last_callback_time = rclcpp::Clock().now();
    if (msg.buttons[0] == 1)
      running_state = 1;
    if (msg.buttons[1] == 1)
      running_state = 2;
    control_loop->setHeight(lift_height);
    control_loop->setChassisCmd(msg.axes[4] * 2, msg.axes[3] * 2,
                                msg.axes[0] * 4, running_state);
  });
  rclcpp::Rate loop_rate(500);
  while (rclcpp::ok()) {
    control_loop->loop();
    spin_some(node);
    loop_rate.sleep();
  }
  return 0;
}
