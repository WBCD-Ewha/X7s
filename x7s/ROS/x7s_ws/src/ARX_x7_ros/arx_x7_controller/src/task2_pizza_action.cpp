#include "arx_x7_controller/x7_controller.hpp"
#include <Eigen/Dense>
#include <ros/ros.h>
#include <thread>
#include <chrono>

void sleep_sec(double t) {
  std::this_thread::sleep_for(std::chrono::duration<double>(t));
}

void graspPizza(
    arx::x7::X7Controller& arm,
    const Eigen::Isometry3d& pose_cam,
    const Eigen::Matrix4d& camera_extrinsic,
    const Eigen::Quaterniond& fixed_quat
) {
  // 1. Start pose
  std::vector<double> start_config(7, 0.0);
  arm.setJointPositions(start_config);
  sleep_sec(1.5);

  // 2. camera frame → world frame
  Eigen::Matrix4d cam_T_world = camera_extrinsic.inverse();
  Eigen::Isometry3d pose_world = Eigen::Isometry3d(cam_T_world) * pose_cam;

  ROS_INFO_STREAM("pose in world:\n" << pose_world.matrix());

  // 3. Gripper 열기
  arm.setCatch(0.2);
  sleep_sec(1.0);

  // 4. Grasp pose로 이동 (orientation은 fixed_approach_quat 사용)
  pose_world.linear() = fixed_approach_quat.toRotationMatrix();

  arm.setEndPose(pose_world);
  sleep_sec(2.0);

  ROS_INFO("Grasp the plate.");

  // 5. Gripper 닫기
  arm.setCatch(0.0);
  sleep_sec(1.0);

  ROS_INFO("Complete.");
}

movetoContainer(
    arx::x7::X7Controller& arm,
    const Eigen::Isometry3d& pose_cam,
    const Eigen::Matrix4d& camera_extrinsic,
    const std::pair<Eigen::Quaterniond, Eigen::Quaterniond>& fixed_approach_quat
    double angle_rad
) {

  ee_pose = arm.getEndPose()
  print("ee_pose: ", ee_pose)

  joint_curr = arm.getJointCurrent()
  print("joint_currents: ", joint_curr)

  // 2. camera frame → world frame
  Eigen::Matrix4d cam_T_world = camera_extrinsic.inverse();
  Eigen::Isometry3d pose_world = Eigen::Isometry3d(cam_T_world) * pose_cam;

  ROS_INFO_STREAM("pose in world:\n" << pose_world.matrix());

  // 4. Grasp pose로 이동 (orientation은 fixed_approach_quat 사용)
  pose_world.linear() = fixed_approach_quat.first.toRotationMatrix();

  arm.setEndPose(pose_world);
  sleep_sec(2.0);

  Eigen::Matrix3d rot = pose_world.linear();
  Eigen::AngleAxisd roll_rot(angle_rad, rot.col(1));  // gripper의 Y축 = col(1)
  pose_world.linear() = roll_rot.toRotationMatrix() * rot;

  ROS_INFO_STREAM("Pose after Y-axis tilt:\n" << pose_world.matrix());
  arm.setEndPose(pose_world);
  sleep_sec(2.0);


}


int main(int argc, char** argv) {
  ros::init(argc, argv, "pick_and_place_pizza_node");
  ros::NodeHandle nh_left("left");
  ros::NodeHandle nh_right("right");

  arx::x7::X7Controller left_arm(nh_left);
  arx::x7::X7Controller right_arm(nh_right);

   // ===== 1. Grasp plate pose =====
  Eigen::Isometry3d object_pose_cam = Eigen::Isometry3d::Identity();
  object_pose_cam.translate(Eigen::Vector3d(0.3, 0.2, 0.45));  // 예: Y > 0 → 왼팔 사용

   // ===== 2. Goal pose =====
   Eigen::Isometry3d goal_pose_cam = Eigen::Isometry3d::Identity();
   goal_pose_cam.translate(Eigen::Vector3d(0.4, 0.2, 0.6));  // 컨테이너 위쪽 위치

  // 고정 orientation (wxyz)
  Eigen::Quaterniond q_fixed(1.0, 0.0, 0.0, 0.0);

  // 카메라 extrinsic (실제 값으로 교체 필요)
  Eigen::Matrix4d camera_extrinsic = Eigen::Matrix4d::Identity();

  double angle_deg = 80
  double angle_rad = angle_deg * M_PI / 180.0;

  // 기준에 따라 팔 선택 (Y값 기준)
  bool use_left = object_pose_cam.translation().y() > 0;

  if (use_left) {
    ROS_INFO("Using LEFT arm to grasp.");
    graspPlate(left_arm, object_pose_cam, camera_extrinsic, q_fixed);
    moveToContainer(left_arm, goal_pose_cam, camera_extrinsic, q_fixed, angle_rad);
  } else {
    ROS_INFO("Using RIGHT arm to grasp.");
    graspPlate(right_arm, object_pose_cam, camera_extrinsic, q_fixed);
    moveToContainer(right_arm, goal_pose_cam, camera_extrinsic, q_fixed, angle_rad * (-1.0));
  }

  return 0;
}