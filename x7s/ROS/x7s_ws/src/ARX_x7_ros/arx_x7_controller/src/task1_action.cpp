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

void grasp_cloth(X7StateInterface& controller, ros::NodeHandle& nh, const Eigen::Matrix4d& left_cam_pose, const Eigen::Matrix4d& right_cam_pose, const Eigen::Matrix4d& camera_extrinsic, const std::vector<double>& left_rpy, const std::vector<double>& right_rpy, double pick_up_height = 0.1) {
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

void stretch_cloth_force(X7StateInterface& controller, ros::NodeHandle& nh, double max_torque_threshold = 1.2, double step_size = 0.02) {
    while (ros::ok()) {
        std::vector<double> left_pose = controller.get_latest_ee_pose(true);
        std::vector<double> right_pose = controller.get_latest_ee_pose(false);
        std::vector<double> left_joint = controller.get_latest_joint_info(true);
        std::vector<double> right_joint = controller.get_latest_joint_info(false);

        // torque 정보는 인덱스 8~15 (총 8개)
        std::vector<double> left_torque(left_joint.begin() + 8, left_joint.begin() + 16);
        std::vector<double> right_torque(right_joint.begin() + 8, right_joint.begin() + 16);

        ROS_INFO("left current pose: ");
        for (auto val : left_pose) std::cout << val << " ";
        std::cout << std::endl;

        ROS_INFO("right current pose: ");
        for (auto val : right_pose) std::cout << val << " ";
        std::cout << std::endl;

        ROS_INFO("left_joint current torque: ");
        for (auto val : left_torque) std::cout << val << " ";
        std::cout << std::endl;

        ROS_INFO("right_joint current torque: ");
        for (auto val : right_torque) std::cout << val << " ";
        std::cout << std::endl;

        bool over_threshold = false;
        for (auto& val : left_torque) if (val > max_torque_threshold) over_threshold = true;
        for (auto& val : right_torque) if (val > max_torque_threshold) over_threshold = true;
        //if (over_threshold) break;

        left_pose[1] += step_size;
        right_pose[1] -= step_size;

        std::thread left_thread(&X7StateInterface::set_ee_pose_cmd, &controller, std::ref(nh), true, left_pose, 0.0);
        std::thread right_thread(&X7StateInterface::set_ee_pose_cmd, &controller, std::ref(nh), false, right_pose, 0.0);
        left_thread.join();
        right_thread.join();

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    }
}

void stretch_cloth_manual(X7StateInterface& controller, ros::NodeHandle& nh, double step_size = 0.08) {
    //while (ros::ok()) {
        std::vector<double> left_pose = controller.get_latest_ee_pose(true);
        std::vector<double> right_pose = controller.get_latest_ee_pose(false);

        ROS_INFO("left current pose: ");
        for (auto val : left_pose) std::cout << val << " ";
        std::cout << std::endl;

        ROS_INFO("right current pose: ");
        for (auto val : right_pose) std::cout << val << " ";
        std::cout << std::endl;

        left_pose[1] += step_size;
        right_pose[1] -= step_size;

        std::thread left_thread(&X7StateInterface::set_ee_pose_cmd, &controller, std::ref(nh), true, left_pose, 0.0);
        std::thread right_thread(&X7StateInterface::set_ee_pose_cmd, &controller, std::ref(nh), false, right_pose, 0.0);
        left_thread.join();
        right_thread.join();

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    //}
}

void fling_cloth(X7StateInterface& controller, ros::NodeHandle& nh,
                 const std::vector<double>& left_fling_pose, const std::vector<double>& right_fling_pose) {
std::vector<double> left_joint = controller.get_latest_joint_info(true);
    std::vector<double> right_joint = controller.get_latest_joint_info(false);

    std::vector<double> left_pos(left_joint.begin(), left_joint.begin() + 8);
    std::vector<double> right_pos(right_joint.begin(), right_joint.begin() + 8);

    ROS_INFO_STREAM("left_joint current pos:");
    for (auto val : left_pos) ROS_INFO_STREAM(val);

    ROS_INFO_STREAM("right_joint current pos:");
    for (auto val : right_pos) ROS_INFO_STREAM(val);

    std::vector<double> left_fling_pose_ = left_fling_pose;
    std::vector<double> right_fling_pose_ = right_fling_pose;

    // fling 동작
    std::thread left_thread([&controller, &nh, left_fling_pose_]() {
        controller.set_ee_pose_cmd(nh, true, left_fling_pose_, 3.0);
    });

    std::thread right_thread([&controller, &nh, right_fling_pose_]() {
        controller.set_ee_pose_cmd(nh, false, right_fling_pose_, 3.0);
    });

    left_thread.join();
    right_thread.join();
    ROS_INFO("Fling the cloth complete");

    // 복귀 동작
    std::thread left_return([&controller, &nh, left_pos]() {
        controller.set_joint_pos_cmd(nh, true, left_pos);
    });

    std::thread right_return([&controller, &nh, right_pos]() {
        controller.set_joint_pos_cmd(nh, false, right_pos);
    });

    left_return.join();
    right_return.join();
    ROS_INFO("Place the cloth complete");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "task1_action");
    ros::NodeHandle nh;
    
    // test
    //std::vector<double> test_pos_left = {-0.361, -1.15, -0.05, 0.0157, 0.471, -0.189, 1.0, 0.0};
    //std::vector<double> test_pos_right = {0.361, 1.15, -0.05, 0.0157, -0.471, 0.189, -1.0, 0.0};
    
    //std::thread left_thread(&X7StateInterface::set_joint_pos_cmd, &controller, std::ref(nh), true, test_pos_left);
    //std::thread right_thread(&X7StateInterface::set_joint_pos_cmd, &controller, std::ref(nh), false, test_pos_right);
    //left_thread.join();
    //right_thread.join();
    
    //std::vector<double> current_left_pose, current_right_pose;
    //get_current_poses(controller, current_left_pose, current_right_pose);
    //

    // 단일 controller 인스턴스로 양팔 제어
    arx::x7::X7StateInterface controller(nh);
    
    int num = 0;
    while (ros::ok() && num < 3) {
        ROS_INFO_STREAM("Starting loop iteration: " << num);

        // Example pose from camera (homogeneous transform 4x4)
        Eigen::Matrix4d left_cam_pose = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d right_cam_pose = Eigen::Matrix4d::Identity();

        std::vector<double> left, right;
        int loop_num;
        std::tie(left, right, loop_num) = controller.get_pose_loop();
        left_cam_pose(0, 3) = left[0];
        left_cam_pose(1, 3) = left[1];
        left_cam_pose(2, 3) = left[2];
        right_cam_pose(0, 3) = right[0];
        right_cam_pose(1, 3) = right[1];
        right_cam_pose(2, 3) = right[2];

        Eigen::Matrix4d camera_extrinsic = Eigen::Matrix4d::Identity(); // TODO: 실제 Extrinsic

        std::vector<double> left_quat_rpy = {1.5707963, 1.5707963, 0};
        std::vector<double> right_quat_rpy = {-1.5707963, 1.5707963, 0};
        double pick_up_height = 0.2;

        grasp_cloth(controller, nh, left_cam_pose, right_cam_pose, camera_extrinsic, left_quat_rpy, right_quat_rpy, pick_up_height);

        stretch_cloth_manual(controller, nh, 0.08);  // TODO: stretch 값 정하기

        std::vector<double> left_fling_pose = {0.25, 0.10, 0.226, 0.0, 0.0, 0.0, 0.0};   // TODO: 적절히 조정
        std::vector<double> right_fling_pose = {0.25, -0.10, 0.226, 0.0, 0.0, 0.0, 0.0};   // TODO: 적절히 조정

        fling_cloth(controller, nh, left_fling_pose, right_fling_pose);

        // check if action was successful
        bool success = controller.is_action_fin();

        if (success) {
            ROS_INFO_STREAM("Action loop " << num << " succeeded.");
            num += 1;
        } else {
            ROS_WARN_STREAM("Action loop " << num << " failed. Retrying.");
        }

        ros::Duration(2.0).sleep();  // optional cooldown
    }

    return 0;
}
