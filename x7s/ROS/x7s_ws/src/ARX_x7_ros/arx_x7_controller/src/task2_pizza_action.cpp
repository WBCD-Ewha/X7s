#include "arx_x7_controller/x7_controller.hpp"
#include <Eigen/Dense>
#include <ros/ros.h>
#include <thread>
#include <chrono>

void sleep_sec(double t) {
  std::this_thread::sleep_for(std::chrono::duration<double>(t));
}

void graspPizza(
    arx::x7::X7Controller& left_arm,
    arx::x7::X7Controller& right_arm,
    const Eigen::Isometry3d& left_pose_cam,
    const Eigen::Isometry3d& right_pose_cam,
    const Eigen::Matrix4d& camera_extrinsic,
    const std::pair<Eigen::Quaterniond, Eigen::Quaterniond>& fixed_approach_quat
) {
  // 1. Start pose
  std::vector<double> left_start_config(7, 0.0);
  std::vector<double> right_start_config(7, 0.0);
  left_arm.setJointPositions(left_start_config);
  right_arm.setJointPositions(right_start_config);
  sleep_sec(1.5);

  // 2. camera frame → world frame
  Eigen::Matrix4d cam_T_world = camera_extrinsic.inverse();
  Eigen::Isometry3d left_pose_world = Eigen::Isometry3d(cam_T_world) * left_pose_cam;
  Eigen::Isometry3d right_pose_world = Eigen::Isometry3d(cam_T_world) * right_pose_cam;

  ROS_INFO_STREAM("Left pose in world:\n" << left_pose_world.matrix());
  ROS_INFO_STREAM("Right pose in world:\n" << right_pose_world.matrix());

  // 3. Gripper 열기
  left_arm.setCatch(0.2);
  right_arm.setCatch(0.2);
  sleep_sec(1.0);

  // 4. Grasp pose로 이동 (orientation은 fixed_approach_quat 사용)
  left_pose_world.linear() = fixed_approach_quat.first.toRotationMatrix();
  right_pose_world.linear() = fixed_approach_quat.second.toRotationMatrix();

  left_arm.setEndPose(left_pose_world);
  right_arm.setEndPose(right_pose_world);
  sleep_sec(2.0);

  ROS_INFO("Grasp the plate.");

  // 5. Gripper 닫기
  left_arm.setCatch(0.0);
  right_arm.setCatch(0.0);
  sleep_sec(1.0);

  ROS_INFO("Complete.");
}

movetoContainer(
    arx::x7::X7Controller& left_arm,
    arx::x7::X7Controller& right_arm,
    const Eigen::Isometry3d& left_pose_cam,
    const Eigen::Isometry3d& right_pose_cam,
    const Eigen::Matrix4d& camera_extrinsic,
    const std::pair<Eigen::Quaterniond, Eigen::Quaterniond>& fixed_approach_quat
) {

  left_ee_pose = left_arm.getEndPose()
  right_ee_pose = right_arm.getEndPose()
  print("left_ee_pose: ", left_ee_pose)
  print("right_ee_pose: ", right_ee_pose)

  left_joint_curr = left_arm.getJointCurrent()
  right_joint_curr = right_arm.getJointCurrent()
  print("left_joint_currents: ", left_joint_curr)
  print("right_joint_currents: ", right_joint_curr)

  // 2. camera frame → world frame
  Eigen::Matrix4d cam_T_world = camera_extrinsic.inverse();
  Eigen::Isometry3d left_pose_world = Eigen::Isometry3d(cam_T_world) * left_pose_cam;
  Eigen::Isometry3d right_pose_world = Eigen::Isometry3d(cam_T_world) * right_pose_cam;

  ROS_INFO_STREAM("Left pose in world:\n" << left_pose_world.matrix());
  ROS_INFO_STREAM("Right pose in world:\n" << right_pose_world.matrix());

  // 4. Grasp pose로 이동 (orientation은 fixed_approach_quat 사용)
  left_pose_world.linear() = fixed_approach_quat.first.toRotationMatrix();
  right_pose_world.linear() = fixed_approach_quat.second.toRotationMatrix();

  left_arm.setEndPose(left_pose_world);
  right_arm.setEndPose(right_pose_world);
  sleep_sec(2.0);


}


int main(int argc, char** argv) {
  ros::init(argc, argv, "grasp_pizza_node");
  ros::NodeHandle nh_left("~left");
  ros::NodeHandle nh_right("~right");

  arx::x7::X7Controller left_arm(nh_left);   // arm_type=0
  arx::x7::X7Controller right_arm(nh_right); // arm_type=1

  // 예시 grasp pose (camera frame 기준)
  Eigen::Isometry3d left_pose_cam = Eigen::Isometry3d::Identity();
  left_pose_cam.translate(Eigen::Vector3d(0.3, 0.2, 0.45));

  Eigen::Isometry3d right_pose_cam = Eigen::Isometry3d::Identity();
  right_pose_cam.translate(Eigen::Vector3d(0.3, -0.2, 0.45));

  // 예시 고정 orientation (wxyz)
  Eigen::Quaterniond q_left(1.0, 0.0, 0.0, 0.0);
  Eigen::Quaterniond q_right(1.0, 0.0, 0.0, 0.0);

  // 예시 camera extrinsic
  Eigen::Matrix4d camera_extrinsic = Eigen::Matrix4d::Identity(); // 실제 값으로 대체

  graspPlate(left_arm, right_arm, left_pose_cam, right_pose_cam, camera_extrinsic, {q_left, q_right});
  movetoContainer(left_arm, right_arm, left_pose_cam, right_pose_cam, camera_extrinsic, {q_left, q_right})
  return 0;
}