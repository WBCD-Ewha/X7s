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

void grasp_and_place(ros::NodeHandle& nh, bool is_left,
                     const Eigen::Isometry3d& object_pose_cam,
                     const Eigen::Isometry3d& goal_pose_cam,
                     const Eigen::Matrix4d& camera_extrinsic,
                     const Eigen::Quaterniond& fixed_quat,
                     double angle_rad) {

  // 1. 카메라 → 월드 좌표계 변환
  Eigen::Matrix4d cam_T_world = camera_extrinsic.inverse();
  Eigen::Isometry3d object_pose_world = Eigen::Isometry3d(cam_T_world) * object_pose_cam;
  Eigen::Isometry3d goal_pose_world = Eigen::Isometry3d(cam_T_world) * goal_pose_cam;

  // 2. 고정 orientation 적용
  object_pose_world.linear() = fixed_quat.toRotationMatrix();
  goal_pose_world.linear() = fixed_quat.toRotationMatrix();

  // 3. Gripper 열기
  // TODO: check yaw orientation
  set_ee_pose_cmd(nh, is_left,
                  {object_pose_world.translation().x(),
                   object_pose_world.translation().y(),
                   object_pose_world.translation().z(),
                   0.0, 0.0, 0.0},
                  3.0); // open
  sleep_sec(1.0);

  // 4. Grasp pose 이동
  set_ee_pose_cmd(nh, is_left,
                  {object_pose_world.translation().x(),
                   object_pose_world.translation().y(),
                   object_pose_world.translation().z(),
                   0.0, 0.0, -1.0},
                  3.0);
  sleep_sec(2.0);

  // 5. Gripper 닫기
  set_ee_pose_cmd(nh, is_left,
                  {object_pose_world.translation().x(),
                   object_pose_world.translation().y(),
                   object_pose_world.translation().z(),
                   0.0, 0.0, -1.0},
                  0.0); // close
  sleep_sec(1.0);

  // 6. Goal pose로 이동
  // TODO : check roll orientation
  set_ee_pose_cmd(nh, is_left,
                  {goal_pose_world.translation().x(),
                   goal_pose_world.translation().y(),
                   goal_pose_world.translation().z(),
                   -1.0, 0.0, -1.0},
                  0.0);
  sleep_sec(2.0);

  // 7. Goal pose에서 Y축 회전 적용
  Eigen::Matrix3d rot = goal_pose_world.linear();
  Eigen::AngleAxisd y_rot(angle_rad, rot.col(1));
  Eigen::Matrix3d rotated = y_rot.toRotationMatrix() * rot;

  Eigen::Vector3d euler = rotated.eulerAngles(0, 1, 2);  // RPY

  set_ee_pose_cmd(nh, is_left,
                  {goal_pose_world.translation().x(),
                   goal_pose_world.translation().y(),
                   goal_pose_world.translation().z(),
                   euler[0], euler[1], euler[2]},
                  0.0);
  sleep_sec(2.0);

  // TODO : 다시 plate 놓는 코드 필요 (return to obj pose)
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pick_and_place_node");
  ros::NodeHandle nh;

  // 1. grasp pose
  Eigen::Isometry3d object_pose_cam = Eigen::Isometry3d::Identity();
  object_pose_cam.translate(Eigen::Vector3d(0.05, 0.1, 0.2));  // Y 기준 왼팔 후보

  // 2. Goal pose (컨테이너 위)
  Eigen::Isometry3d goal_pose_cam = Eigen::Isometry3d::Identity();
  goal_pose_cam.translate(Eigen::Vector3d(0.05, -0.1, 0.4));

  // 3. 고정 orientation (wxyz)
  Eigen::Quaterniond fixed_quat(1.0, 0.0, 0.0, 0.0);

  // 4. 카메라 extrinsic
  // TODO: check Extrinsic
  Eigen::Matrix4d camera_extrinsic = Eigen::Matrix4d::Identity();

  // 5. 카메라 → 월드 변환 후 좌표계 기준으로 팔 선택
  Eigen::Matrix4d cam_T_world = camera_extrinsic.inverse();
  Eigen::Isometry3d object_pose_world = Eigen::Isometry3d(cam_T_world) * object_pose_cam;
  bool use_left = object_pose_world.translation().y() > 0;

  ROS_INFO_STREAM("Using " << (use_left ? "LEFT" : "RIGHT") << " arm");

  // 6. 회전 각도 (Y축 기준)
  double angle_deg = 80.0;
  double angle_rad = angle_deg * M_PI / 180.0;

  // 7. 전체 수행
  grasp_and_place(nh, use_left, object_pose_cam, goal_pose_cam, camera_extrinsic, fixed_quat, use_left ? angle_rad : -angle_rad);

  return 0;
}
