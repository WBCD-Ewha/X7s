#include "arx_x7_controller/x7_controller.hpp"
#include <ros/package.h>
#include "ros/ros.h"
#include <arm_control/PosCmd.h>
#include <Eigen/Dense>
#include <thread>
#include <chrono>
#include <vector>

void sleep_sec(double t) {
  std::this_thread::sleep_for(std::chrono::duration<double>(t));
}

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

void move_arm(ros::NodeHandle& nh, bool is_left,
              const Eigen::Isometry3d& grasp_pose_cam,
              const Eigen::Isometry3d& goal_pose_cam,
              const Eigen::Quaterniond& fixed_quat,
              const Eigen::Matrix4d& camera_extrinsic,
              double press_offset) {
  Eigen::Matrix4d cam_T_world = camera_extrinsic.inverse();
  Eigen::Isometry3d grasp_pose_world = Eigen::Isometry3d(cam_T_world) * grasp_pose_cam;
  Eigen::Isometry3d goal_pose_world = Eigen::Isometry3d(cam_T_world) * goal_pose_cam;

  // 고정 orientation 적용
  grasp_pose_world.linear() = fixed_quat.toRotationMatrix();
  goal_pose_world.linear() = fixed_quat.toRotationMatrix();

  // Grasp pose로 이동
  set_ee_pose_cmd(nh, is_left,
                  {grasp_pose_world.translation().x(),
                   grasp_pose_world.translation().y(),
                   grasp_pose_world.translation().z(),
                   0.0, 0.0, 0.0}, 3.0);
  sleep_sec(2.0);

  // Gripper 닫기
  set_ee_pose_cmd(nh, is_left,
                  {grasp_pose_world.translation().x(),
                   grasp_pose_world.translation().y(),
                   grasp_pose_world.translation().z(),
                   0.0, 0.0, 0.0}, 0.0);
  sleep_sec(1.0);

  // Goal pose로 이동
  set_ee_pose_cmd(nh, is_left,
                  {goal_pose_world.translation().x(),
                   goal_pose_world.translation().y(),
                   goal_pose_world.translation().z(),
                   0.0, 0.0, 0.0}, 0.0);
  sleep_sec(2.0);

  // Z축 방향으로 누르기
  Eigen::Vector3d z_axis = goal_pose_world.linear().col(2);  // world 기준 z축
  Eigen::Vector3d push_position = goal_pose_world.translation() - press_offset * z_axis;

  set_ee_pose_cmd(nh, is_left,
                  {push_position.x(), push_position.y(), push_position.z(),
                   0.0, 0.0, 0.0}, 0.0);
  sleep_sec(2.0);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "close_lid_node");
  ros::NodeHandle nh;

  // 1. grasp pose
  Eigen::Isometry3d left_grasp_pose = Eigen::Isometry3d::Identity();
  left_grasp_pose.translate(Eigen::Vector3d(0.05, 0.1 0.01));

  Eigen::Isometry3d right_grasp_pose = Eigen::Isometry3d::Identity();
  right_grasp_pose.translate(Eigen::Vector3d(0.05, 0.0, 0.01));

  // 2. goal pose
  Eigen::Isometry3d left_goal_pose = left_grasp_pose;
  left_goal_pose.translate(Eigen::Vector3d(0.05, 0.0, 0.15));

  Eigen::Isometry3d right_goal_pose = right_grasp_pose;
  right_goal_pose.translate(Eigen::Vector3d(0.05, -0.1, 0.15));

  // 3. 고정 orientation
  Eigen::Quaterniond q_left(1.0, 0.0, 0.0, 0.0);
  Eigen::Quaterniond q_right(1.0, 0.0, 0.0, 0.0);

  // 4. extrinsic
  Eigen::Matrix4d camera_extrinsic = Eigen::Matrix4d::Identity();  // 실제 환경에 맞게 교체

  double press_offset = 0.1;

  // 5. 왼팔/오른팔 동시 실행
  std::thread left_thread(move_arm, std::ref(nh), true,
                          left_grasp_pose, left_goal_pose, q_left, camera_extrinsic, press_offset);

  std::thread right_thread(move_arm, std::ref(nh), false,
                           right_grasp_pose, right_goal_pose, q_right, camera_extrinsic, press_offset);

  left_thread.join();
  right_thread.join();

  return 0;
}
