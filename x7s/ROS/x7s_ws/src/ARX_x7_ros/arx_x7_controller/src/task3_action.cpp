#include "arx_x7_controller/x7_controller.hpp"
#include <Eigen/Dense>
#include <ros/ros.h>
#include <thread>
#include <chrono>

void sleep_sec(double t) {
  std::this_thread::sleep_for(std::chrono::duration<double>(t));
}

void graspLid(
    arx::x7::X7Controller& left_arm,
    arx::x7::X7Controller& right_arm,
    const Eigen::Isometry3d& left_pose_cam,
    const Eigen::Isometry3d& right_pose_cam,
    const Eigen::Matrix4d& camera_extrinsic,
    const std::pair<Eigen::Quaterniond, Eigen::Quaterniond>& fixed_quats
) {
  std::vector<double> start_config(7, 0.0);
  left_arm.setJointPositions(start_config);
  right_arm.setJointPositions(start_config);
  sleep_sec(1.5);

  Eigen::Matrix4d cam_T_world = camera_extrinsic.inverse();
  Eigen::Isometry3d left_pose_world = Eigen::Isometry3d(cam_T_world) * left_pose_cam;
  Eigen::Isometry3d right_pose_world = Eigen::Isometry3d(cam_T_world) * right_pose_cam;

  left_pose_world.linear() = fixed_quats.first.toRotationMatrix();
  right_pose_world.linear() = fixed_quats.second.toRotationMatrix();

  left_arm.setCatch(0.2);
  right_arm.setCatch(0.2);
  sleep_sec(1.0);

  left_arm.setEndPose(left_pose_world);
  right_arm.setEndPose(right_pose_world);
  sleep_sec(2.0);

  left_arm.setCatch(0.0);
  right_arm.setCatch(0.0);
  sleep_sec(1.0);
}

void moveLid(
    arx::x7::X7Controller& left_arm,
    arx::x7::X7Controller& right_arm,
    const Eigen::Isometry3d& left_goal_pose_cam,
    const Eigen::Isometry3d& right_goal_pose_cam,
    const Eigen::Matrix4d& camera_extrinsic,
    const std::pair<Eigen::Quaterniond, Eigen::Quaterniond>& fixed_quats,
    double press_offset
) {

  left_ee_pose = left_arm.getEndPose()
  print("left_ee_pose: ", left_ee_pose)

  right_ee_pose = right_arm.getEndPose()
  print("right_ee_pose: ", right_ee_pose)

  left_joint_curr = left_arm.getJointCurrent()
  print("left_joint_currents: ", left_joint_curr)

  right_joint_curr = right_arm.getJointCurrent()
  print("right_joint_currents: ", right_joint_curr)

  Eigen::Matrix4d cam_T_world = camera_extrinsic.inverse();
  Eigen::Isometry3d left_pose_world = Eigen::Isometry3d(cam_T_world) * left_goal_pose_cam;
  Eigen::Isometry3d right_pose_world = Eigen::Isometry3d(cam_T_world) * right_goal_pose_cam;

  left_pose_world.linear() = fixed_quats.first.toRotationMatrix();
  right_pose_world.linear() = fixed_quats.second.toRotationMatrix();

  left_arm.setEndPose(left_pose_world);
  right_arm.setEndPose(right_pose_world);
  sleep_sec(2.0);

  // press
  Eigen::Vector3d z_left = left_pose_world.linear().col(2);
  Eigen::Vector3d z_right = right_pose_world.linear().col(2);

  Eigen::Isometry3d left_push_pose = left_pose_world;
  Eigen::Isometry3d right_push_pose = right_pose_world;

  left_push_pose.translation() += -press_offset * z_left;
  right_push_pose.translation() += -press_offset * z_right;

  ROS_INFO_STREAM("Pushing down...");
  left_arm.setEndPose(left_push_pose);
  right_arm.setEndPose(right_push_pose);
  sleep_sec(2.0);

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "close_the_container_node");
  ros::NodeHandle nh_left("left");
  ros::NodeHandle nh_right("right");

  arx::x7::X7Controller left_arm(nh_left);
  arx::x7::X7Controller right_arm(nh_right);

  // 1. bimanual grasp pose
  Eigen::Isometry3d left_grasp_pose = Eigen::Isometry3d::Identity();
  left_grasp_pose.translate(Eigen::Vector3d(0.3, 0.2, 0.45));

  Eigen::Isometry3d right_grasp_pose = Eigen::Isometry3d::Identity();
  right_grasp_pose.translate(Eigen::Vector3d(0.3, -0.2, 0.45));

  // 2. goal pose
  Eigen::Isometry3d left_goal_pose = left_grasp_pose;
  left_goal_pose.translate(Eigen::Vector3d(0.1, 0.0, 0.15));

  Eigen::Isometry3d right_goal_pose = right_grasp_pose;
  right_goal_pose.translate(Eigen::Vector3d(0.1, 0.0, 0.15));

  // 고정 orientation
  Eigen::Quaterniond q_left(1.0, 0.0, 0.0, 0.0);
  Eigen::Quaterniond q_right(1.0, 0.0, 0.0, 0.0);

  Eigen::Matrix4d camera_extrinsic = Eigen::Matrix4d::Identity();

  double press_offset = 0.1

  ROS_INFO("Grasping the lid with both arms...");
  graspLid(left_arm, right_arm, left_grasp_pose, right_grasp_pose, camera_extrinsic, {q_left, q_right});

  ROS_INFO("Moving to container and tilting...");
  moveLid(left_arm, right_arm, left_goal_pose, right_goal_pose, camera_extrinsic, {q_left, q_right}), press_offset;

  return 0;
}