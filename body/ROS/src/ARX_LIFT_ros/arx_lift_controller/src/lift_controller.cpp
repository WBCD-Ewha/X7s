#include <arm_control/PosCmd.h>
#include <arx_lift_src/lift_head_control_loop.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "lift_controller");
  ros::NodeHandle nh("~");
  ros::Rate loop_rate(500);
  int type = nh.param("robot_type", 0);
  if(type == 0)
  	ROS_INFO("robot_type: lift");
  else
  	ROS_INFO("robot_type: x7s");
  LiftHeadControlLoop control_loop(
      "can5", static_cast<LiftHeadControlLoop::RobotType>(type));
  int running_state = 2;
  double lift_height = 0;
  ros::Subscriber sub = nh.subscribe<arm_control::PosCmd>(
      "/ARX_VR_L", 1, [&](const arm_control::PosCmd::ConstPtr &msg) {
        control_loop.setHeight(msg->height / 41.54);
        control_loop.setWaistPos(msg->tempFloatData[0]);
        control_loop.setHeadYaw(msg->head_yaw);
        control_loop.setHeadPitch(-msg->head_pit);
        if (type == 0)
          control_loop.setChassisCmd(msg->chx * 1/2.5, -msg->chy * 1/2.5,
                                     msg->chz * 1/2.5, msg->mode1);
        else
          control_loop.setChassisCmd(msg->chx *1/5, -msg->chy * 1 / 5, msg->chz * 1 / 5,
                                     msg->mode1);
      });
  ros::Time last_callback_time = ros::Time::now();
  ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>(
      "/joy", 1, [&](const sensor_msgs::Joy::ConstPtr &msg) {
        lift_height +=
            msg->axes[1] * (ros::Time::now() - last_callback_time).toSec();
        last_callback_time = ros::Time::now();
        if (msg->buttons[0] == 1)
          running_state = 1;
        if (msg->buttons[1] == 1)
          running_state = 2;
        control_loop.setHeight(lift_height);
        control_loop.setChassisCmd(msg->axes[4] * 2, msg->axes[3] * 2,
                                   msg->axes[0] * 4, running_state);
      });
  while (ros::ok()) {
    control_loop.loop();
    ros::spinOnce();
    loop_rate.sleep();
  }
}
