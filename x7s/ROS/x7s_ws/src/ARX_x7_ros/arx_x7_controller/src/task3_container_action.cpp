#include "arx_x7_controller/x7_controller.hpp"
#include "arx_x7_controller/ros_control.hpp"  // don't forget
#include "ros/ros.h"
#include <thread>
#include <vector>
#include <arm_control/PosCmd.h>
#include <arm_control/JointControl.h>
#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include <ros/package.h>
#include <chrono>

using namespace arx::x7;

// camera frame -> world frame transformation
Eigen::Matrix4d transform_camera_to_world(const Eigen::Matrix4d& cam_pose, const Eigen::Matrix4d& camera_extrinsic) {
    return camera_extrinsic.inverse() * cam_pose;
}

// 4x4 matrix -> xyzrpy
void mat_to_xyzrpy(const Eigen::Matrix4d& pose, std::vector<double>& xyz, std::vector<double>& rpy) {
    xyz = {pose(0, 3), pose(1, 3), pose(2, 3)};
    Eigen::Matrix3d rot = pose.block<3, 3>(0, 0);
    Eigen::Vector3d euler = rot.eulerAngles(2, 1, 0);  // ZYX
    rpy = {euler[2], euler[1], euler[0]};
}

void move_to_start_pose(X7StateInterface& controller, ros::NodeHandle& nh) {
    std::vector<double> start_config = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::thread left_thread(&X7StateInterface::set_joint_pos_cmd, &controller, std::ref(nh), true, start_config);
    std::thread right_thread(&X7StateInterface::set_joint_pos_cmd, &controller, std::ref(nh), false, start_config);
    left_thread.join();
    right_thread.join();
}

void get_current_poses(X7StateInterface& controller,
                       std::vector<double>& current_left_pose,
                       std::vector<double>& current_right_pose,
                       int max_wait_count = 50) {
    ros::Rate wait_rate(10);
    int wait_count = 0;

    while (wait_count < max_wait_count) {
        ros::spinOnce();
        current_left_pose = controller.get_latest_ee_pose(true);
        current_right_pose = controller.get_latest_ee_pose(false);

        bool is_left_valid = !std::all_of(current_left_pose.begin(), current_left_pose.end(),
                                          [](double v) { return std::abs(v) < 1e-5; });
        bool is_right_valid = !std::all_of(current_right_pose.begin(), current_right_pose.end(),
                                           [](double v) { return std::abs(v) < 1e-5; });

        if (is_left_valid && is_right_valid) break;

        wait_rate.sleep();
        ++wait_count;
    }

    if (wait_count == max_wait_count) {
        ROS_WARN("Timeout: ee_pose messages not received from both arms.");
    }

    ROS_INFO("Current Left Pose (x y z r p y):");
    for (double v : current_left_pose) std::cout << v << " ";
    std::cout << std::endl;

    ROS_INFO("Current Right Pose (x y z r p y):");
    for (double v : current_right_pose) std::cout << v << " ";
    std::cout << std::endl;
}


void open_grippers(X7StateInterface& controller, ros::NodeHandle& nh) {
    /// 각 arm의 현재 pose 가져올 것 (x, y, z, r, p, y 형태)
    std::vector<double> current_left_pose, current_right_pose;
    get_current_poses(controller, current_left_pose, current_right_pose);

    std::thread left_thread(&X7StateInterface::set_ee_pose_cmd, &controller,
                            std::ref(nh), true, current_left_pose, 3.0);
    std::thread right_thread(&X7StateInterface::set_ee_pose_cmd, &controller,
                             std::ref(nh), false, current_right_pose, 3.0);
    left_thread.join();
    right_thread.join();
}

void close_grippers(X7StateInterface& controller, ros::NodeHandle& nh) {
    /// 각 arm의 현재 pose 가져올 것 (x, y, z, r, p, y 형태)
    std::vector<double> current_left_pose, current_right_pose;
    get_current_poses(controller, current_left_pose, current_right_pose);

    std::thread left_thread(&X7StateInterface::set_ee_pose_cmd, &controller,
                            std::ref(nh), true, current_left_pose, 0.0);
    std::thread right_thread(&X7StateInterface::set_ee_pose_cmd, &controller,
                             std::ref(nh), false, current_right_pose, 0.0);
    left_thread.join();
    right_thread.join();
}

void grasp_lid(X7StateInterface& controller, ros::NodeHandle& nh, const Eigen::Matrix4d& left_cam_pose, const Eigen::Matrix4d& right_cam_pose, const Eigen::Matrix4d& camera_extrinsic, const std::vector<double>& left_rpy, const std::vector<double>& right_rpy, double pick_up_height) {

    move_to_start_pose(controller, nh);

    // grasp pose transformation w.r.t world frame
    Eigen::Matrix4d left_world = transform_camera_to_world(left_cam_pose, camera_extrinsic);
    Eigen::Matrix4d right_world = transform_camera_to_world(right_cam_pose, camera_extrinsic);

    // matrix conversion to xyzrpy
    std::vector<double> left_xyz, right_xyz, left_rpy_, right_rpy_;
    mat_to_xyzrpy(left_world, left_xyz, left_rpy_);
    mat_to_xyzrpy(right_world, right_xyz, right_rpy_);

    // open grippers
    open_grippers(controller, nh);

    // move to grasp pose
    std::vector<double> left_grasp = {left_xyz[0], left_xyz[1], left_xyz[2], left_rpy[0], left_rpy[1], left_rpy[2]};
    std::vector<double> right_grasp = {right_xyz[0], right_xyz[1], right_xyz[2], right_rpy[0], right_rpy[1], right_rpy[2]};
    std::thread left_thread(&X7StateInterface::set_ee_pose_cmd, &controller, std::ref(nh), true, left_grasp, 3.0);
    std::thread right_thread(&X7StateInterface::set_ee_pose_cmd, &controller, std::ref(nh), false, right_grasp, 3.0);
    left_thread.join();
    right_thread.join();

    // close grippers
    close_grippers(controller, nh);

    // move up a little bit
    left_grasp[2] += pick_up_height;
    right_grasp[2] += pick_up_height;
    std::thread lift_left(&X7StateInterface::set_ee_pose_cmd, &controller, std::ref(nh), true, left_grasp, 0.0);
    std::thread lift_right(&X7StateInterface::set_ee_pose_cmd, &controller, std::ref(nh), false, right_grasp, 0.0);
    lift_left.join();
    lift_right.join();
}

void close_container(X7StateInterface& controller, ros::NodeHandle& nh, const Eigen::Matrix4d& left_container_pose, const Eigen::Matrix4d& right_container_pose, const Eigen::Matrix4d& camera_extrinsic, const std::vector<double>& left_rpy, const std::vector<double>& right_rpy, double push_height) {

    // grasp pose transformation w.r.t world frame
    Eigen::Matrix4d left_world = transform_camera_to_world(left_container_pose, camera_extrinsic);
    Eigen::Matrix4d right_world = transform_camera_to_world(right_container_pose, camera_extrinsic);

    // matrix conversion to xyzrpy
    std::vector<double> left_xyz, right_xyz, left_rpy_, right_rpy_;
    mat_to_xyzrpy(left_world, left_xyz, left_rpy_);
    mat_to_xyzrpy(right_world, right_xyz, right_rpy_);

    std::vector<double> left_pose = controller.get_latest_ee_pose(true);
    std::vector<double> right_pose = controller.get_latest_ee_pose(false);

    // move to grasp pose
    std::vector<double> left_grasp = {left_xyz[0], left_xyz[1], left_pose[2], left_rpy[0], left_rpy[1], left_rpy[2]};
    std::vector<double> right_grasp = {right_xyz[0], right_xyz[1], right_pose[2], right_rpy[0], right_rpy[1], right_rpy[2]};
    std::thread left_thread(&X7StateInterface::set_ee_pose_cmd, &controller, std::ref(nh), true, left_grasp, 0.0);
    std::thread right_thread(&X7StateInterface::set_ee_pose_cmd, &controller, std::ref(nh), false, right_grasp, 0.0);
    left_thread.join();
    right_thread.join();

    // close grippers
    close_grippers(controller, nh);

    // move up a little bit
    left_grasp[2] -= push_height;
    right_grasp[2] -= push_height;
    std::thread lift_left(&X7StateInterface::set_ee_pose_cmd, &controller, std::ref(nh), true, left_grasp, 0.0);
    std::thread lift_right(&X7StateInterface::set_ee_pose_cmd, &controller, std::ref(nh), false, right_grasp, 0.0);
    lift_left.join();
    lift_right.join();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "task3_action");
    ros::NodeHandle nh;

    arx::x7::X7StateInterface controller(nh);


    // TODO: 실제 값으로 대체 (ros service 이용해서 perception node한테서 받기)
    // lid grasp pose
    Eigen::Matrix4d left_cam_pose = Eigen::Matrix4d::Identity();
    left_cam_pose(0, 3) = 0.05;  // x
    left_cam_pose(1, 3) = 0.0;  // y
    left_cam_pose(2, 3) = 0.10;  // z  //TODO

    Eigen::Matrix4d right_cam_pose = Eigen::Matrix4d::Identity();
    right_cam_pose(0, 3) = 0.05;  // x
    right_cam_pose(1, 3) = 0.0;  // y
    right_cam_pose(2, 3) = 0.10;  // z

    Eigen::Matrix4d camera_extrinsic = Eigen::Matrix4d::Identity(); // TODO: 실제 Extrinsic

    double pick_up_height = 0.15;

    // Example rpy orientation (approach)
    std::vector<double> left_quat_rpy = {1.5707963, 1.5707963, 0};
    std::vector<double> right_quat_rpy = {-1.5707963, 1.5707963, 0};

    grasp_lid(controller, nh, left_cam_pose, right_cam_pose, camera_extrinsic, left_quat_rpy, right_quat_rpy, pick_up_height);

    // closing pose
    Eigen::Matrix4d left_container_pose = Eigen::Matrix4d::Identity();
    left_container_pose(0, 3) = 0.05;  // x
    left_container_pose(1, 3) = 0.05;  // y
    left_container_pose(2, 3) = 0.0;  // z  //TODO

    Eigen::Matrix4d right_container_pose = Eigen::Matrix4d::Identity();
    right_container_pose(0, 3) = 0.05;  // x
    right_container_pose(1, 3) = 0.05;  // y
    right_container_pose(2, 3) = 0.0;  // z

    double push_height = 0.15;

    close_container(controller, nh, left_container_pose, right_container_pose, camera_extrinsic, left_quat_rpy, right_quat_rpy, push_height);

    return 0;
}
