#include "x7_controller.hpp"
#include <ros/ros.h>
#include <Eigen/Dense>
#include <thread>
#include <chrono>

using namespace arx::x7;

void sleep(double seconds) {
  std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "execute_grasp_pose_node");
  ros::NodeHandle nh("~");

  X7Controller arm(nh);  // 단일 팔만 제어

  // 1. Perception으로부터 얻은 3D grasp pose
  Eigen::Vector3d grasp_xyz(0.32, 0.15, 0.42);  // 예시: Python에서 추출된 좌표

  // 2. Orientation (쓸 수 있는 경우)
  Eigen::Quaterniond quat(1.0, 0.0, 0.0, 0.0);  // w, x, y, z

  // 3. Isometry로 포즈 구성
  Eigen::Isometry3d grasp_pose = Eigen::Isometry3d::Identity();
  grasp_pose.translate(grasp_xyz);
  grasp_pose.rotate(quat);

  // 4. 실행
  ROS_INFO("Moving to grasp pose...");
  arm.setCatch(0.2);  // 그리퍼 open
  sleep(1.0);

  arm.setEndPose(grasp_pose);  // 위치 이동
  sleep(2.0);

  arm.setCatch(0.0);  // 그리퍼 close
  sleep(1.0);

  // 5. Pick-up 동작 (z축으로 위로 이동)
  grasp_xyz.z() += 0.1;
  grasp_pose.translation() = grasp_xyz;

  arm.setEndPose(grasp_pose);
  sleep(2.0);

  ROS_INFO("Grasp completed.");
  return 0;
}
