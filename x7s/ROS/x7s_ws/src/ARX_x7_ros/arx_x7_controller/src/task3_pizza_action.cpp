#include "arx_x7_controller/x7_controller.hpp"
#include "arx_x7_controller/ros_control.hpp"
#include <ros/package.h>
#include "ros/ros.h"
#include <arm_control/PosCmd.h>
#include <arm_control/JointControl.h>
#include <Eigen/Dense>
#include <thread>
#include <Eigen/Dense>
#include <iostream>
#include <chrono>
#include <vector>
#include <cmath>

using namespace arx::x7;

void sleep_sec(double t) {
  std::this_thread::sleep_for(std::chrono::duration<double>(t));
}

void mat_to_xyzrpy(const Eigen::Matrix4d& pose, std::vector<double>& xyz, std::vector<double>& rpy) {
    xyz = {pose(0, 3), pose(1, 3), pose(2, 3)};
    Eigen::Matrix3d rot = pose.block<3, 3>(0, 0);
    Eigen::Vector3d euler = rot.eulerAngles(2, 1, 0);  // ZYX
    rpy = {euler[2], euler[1], euler[0]};
}

void get_current_poses(X7StateInterface& controller,
                         std::vector<double>& pose,
                         bool is_left,
                         int max_wait_count = 50) {
    ros::Rate wait_rate(10);
    int wait_count = 0;

    while (wait_count < max_wait_count) {
        ros::spinOnce();
        pose = controller.get_latest_ee_pose(is_left);

        bool is_valid = !std::all_of(pose.begin(), pose.end(),
                                     [](double v) { return std::abs(v) < 1e-5; });

        if (is_valid) break;

        wait_rate.sleep();
        ++wait_count;
    }

    if (wait_count == max_wait_count) {
        std::string arm = is_left ? "left" : "right";
        ROS_WARN_STREAM("Timeout: ee_pose message not received from " << arm << " arm.");
    }

    std::string arm = is_left ? "Left" : "Right";
    ROS_INFO_STREAM("Current " << arm << " Pose (x y z r p y):");
    for (double v : pose) std::cout << v << " ";
    std::cout << std::endl;
}

void open_grippers(X7StateInterface& controller, ros::NodeHandle& nh, bool is_left) {
    std::vector<double> pose;
    get_current_poses(controller, pose, is_left);
    controller.set_ee_pose_cmd(nh, is_left, pose, 3.0);
}

void close_grippers(X7StateInterface& controller, ros::NodeHandle& nh, bool is_left) {
    std::vector<double> pose;
    get_current_poses(controller, pose, is_left);
    controller.set_ee_pose_cmd(nh, is_left, pose, 0.0);
}

// camera frame -> world frame transformation
Eigen::Matrix4d transform_camera_to_world(const Eigen::Matrix4d& cam_pose, const Eigen::Matrix4d& camera_extrinsic) {
    return camera_extrinsic.inverse() * cam_pose;
}

void grasp_plate(X7StateInterface& controller, ros::NodeHandle& nh, bool is_left,
                     const Eigen::Matrix4d& object_pose_cam,
                     const Eigen::Matrix4d& camera_extrinsic,
                     const std::vector<double>& grasp_quat_rpy) {

    // cam to world
    Eigen::Matrix4d object_pose_world = transform_camera_to_world(object_pose_cam, camera_extrinsic);

    // matrix conversion to xyzrpy
    std::vector<double> plate_xyz, plate_rpy;
    mat_to_xyzrpy(object_pose_world, plate_xyz, plate_rpy);

    std::vector<double> grasp_plate = {plate_xyz[0], plate_xyz[1], plate_xyz[2], grasp_quat_rpy[0], grasp_quat_rpy[1], grasp_quat_rpy[2]};

    // 1. rpy
    std::vector<double> current_pose;
    get_current_poses(controller, current_pose, is_left);

    std::vector<double> ori_only_pose = {
        current_pose[0], current_pose[1], current_pose[2],
        grasp_quat_rpy[0], grasp_quat_rpy[1], grasp_quat_rpy[2]
    };
    open_grippers(controller, nh, is_left);
    controller.set_ee_pose_cmd(nh, is_left, ori_only_pose, 3.0);

    // 2. position
    // TODO: check orientation
    controller.set_ee_pose_cmd(nh, is_left, grasp_plate, 3.0);

    close_grippers(controller, nh, is_left);
}

void place_pizza(X7StateInterface& controller, ros::NodeHandle& nh, bool is_left,
                     const Eigen::Matrix4d& goal_pose_cam,
                     const Eigen::Matrix4d& camera_extrinsic,
                     const std::vector<double>& goal_quat_rpy,
                     double angle_rad) {

    // cam to world
    Eigen::Matrix4d goal_pose_world = transform_camera_to_world(goal_pose_cam, camera_extrinsic);

    // matrix conversion to xyzrpy
    std::vector<double> goal_xyz, goal_rpy;
    mat_to_xyzrpy(goal_pose_world, goal_xyz, goal_rpy);

    std::vector<double> place_pizza = {goal_xyz[0], goal_xyz[1], goal_xyz[2], goal_quat_rpy[0], goal_quat_rpy[1], goal_quat_rpy[2]};

    // move to goal (place the pizza)
    // TODO: check orientation
    controller.set_ee_pose_cmd(nh, is_left, place_pizza, 0.0);

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pick_and_place_node");
    ros::NodeHandle nh;

    arx::x7::X7StateInterface controller(nh);

    // left : -0.44, -0.691, 0.0943, -0.00157, 1.32, 1.51, -0.44
    // right : 0.44, 0.691, 0.0943, -0.189, -1.32, -1.51, 0.44

    // test
//    std::vector<double> test_pos_left = {-0.44, -0.691, 0.0943, -0.00157, 1.32, 1.51, -0.44};
//    std::vector<double> test_pos_right = {0.44, 0.691, 0.0943, -0.189, -1.32, -1.51, 0.44};
//
//    controller.set_ee_pose_cmd(nh, is_left, test_pos_left, 0.0);
//
//    std::vector<double> current_left_pose
//    get_current_poses(controller, current_left_pose, true);


    // 1. grasp pose
    Eigen::Matrix4d object_pose_cam = Eigen::Matrix4d::Identity();
    object_pose_cam(0, 3) = 0.05;  // x
    object_pose_cam(1, 3) = 0.10;  // y
    object_pose_cam(2, 3) = 0.10;  // z  //TODO

    // 2. Goal pose
    Eigen::Matrix4d goal_pose_cam = Eigen::Matrix4d::Identity();
    goal_pose_cam(0, 3) = 0.05;  // x
    goal_pose_cam(1, 3) = -0.10;  // y
    goal_pose_cam(2, 3) = 0.35;  // z

    // Example rpy orientation (approach)
    // TODO :  check orientation
    std::vector<double> grasp_quat_rpy = {-2.646,  0.042, -2.277};
    std::vector<double> goal_quat_rpy = {-2.646,  0.042, -2.277};

    // 4.extrinsic
    // TODO: check Extrinsic
    Eigen::Matrix4d camera_extrinsic = Eigen::Matrix4d::Identity();

    // 5. choose arm
    Eigen::Matrix4d cam_T_world = camera_extrinsic.inverse();
    Eigen::Matrix4d object_pose_world = Eigen::Matrix4d(cam_T_world) * object_pose_cam;
    Eigen::Vector3d obj_pos = object_pose_world.block<3,1>(0,3);
    bool use_left = obj_pos.x() < 0;

    ROS_INFO_STREAM("Using " << (use_left ? "LEFT" : "RIGHT") << " arm");

    // 6. rotate angle
    double angle_deg = -90.0;
    double angle_rad = angle_deg * M_PI / 180.0;

    // 7. processing
    grasp_plate(controller, nh, use_left, object_pose_cam, camera_extrinsic, grasp_quat_rpy);
    place_pizza(controller, nh, use_left, goal_pose_cam, camera_extrinsic, goal_quat_rpy, angle_rad);

    return 0;
}
