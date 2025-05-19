#include "arx_x7_controller/x7_controller.hpp"
#include "arx_x7_controller/ros_control.hpp"
#include <ros/package.h>
#include "ros/ros.h"
#include <chrono>
#include <thread>
#include <arm_control/PosCmd.h>
#include <arm_control/JointControl.h>
#include <thread>


namespace arx::x7 {

    X7StateInterface::X7StateInterface(ros::NodeHandle& nh) {
    joint_info_sub_left = nh.subscribe("/joint_information", 10, &X7StateInterface::jointInfoCallbackLeft, this);
    ee_pose_sub_left = nh.subscribe("/follow1_pos_back", 10, &X7StateInterface::eePoseCallbackLeft, this);
    joint_info_sub_right = nh.subscribe("/joint_information2", 10, &X7StateInterface::jointInfoCallbackRight, this);
    ee_pose_sub_right = nh.subscribe("/follow2_pos_back", 10, &X7StateInterface::eePoseCallbackRight, this);

    // Service
    single_service = nh.advertiseService("/send_single_pose", &X7StateInterface::singlePoseCallback, this);
    single_moved_service = nh.advertiseService("/send_moved_single_pose", &X7StateInterface::singleMovedPoseCallback, this);
    bimanual_service = nh.advertiseService("/send_bimanual_pose", &X7StateInterface::bimanualPoseCallback, this);
    bimanual_close_service = nh.advertiseService("/send_bimanual_close_pose", &X7StateInterface::bimanualClosePoseCallback, this);
    pose_loop_service = nh.advertiseService("/send_pose_loop", &X7StateInterface::poseLoopCallback, this);
    action_service = nh.advertiseService("/action_fin", &X7StateInterface::actionCallback, this);

    }

    void X7StateInterface::jointInfoCallbackLeft(const arm_control::JointInformation::ConstPtr& msg) {
      latest_joint_info_left = *msg;
    }

    void X7StateInterface::eePoseCallbackLeft(const arm_control::PosCmd::ConstPtr& msg) {
      latest_ee_pose_left = *msg;
    }

    void X7StateInterface::jointInfoCallbackRight(const arm_control::JointInformation::ConstPtr& msg) {
      latest_joint_info_right = *msg;
    }

    void X7StateInterface::eePoseCallbackRight(const arm_control::PosCmd::ConstPtr& msg) {
      latest_ee_pose_right = *msg;
    }

    std::vector<double> X7StateInterface::get_latest_ee_pose(bool is_left) {
        const auto& msg = is_left ? latest_ee_pose_left : latest_ee_pose_right;
        return {msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw};
    }

    std::vector<double> X7StateInterface::get_latest_joint_info(bool is_left) {
    const auto& msg = is_left ? latest_joint_info_left : latest_joint_info_right;
    std::vector<double> joint_info;

    // joint positions
    for (int i = 0; i < 8; ++i) {
        joint_info.push_back(msg.joint_pos[i]);
    }

    // joint velocities
    for (int i = 0; i < 8; ++i) {
        joint_info.push_back(msg.joint_vel[i]);
    }

    // joint currents
    for (int i = 0; i < 8; ++i) {
        joint_info.push_back(msg.joint_cur[i]);
    }

    return joint_info;
}

    // End-effector pose control
    void X7StateInterface::set_ee_pose_cmd(ros::NodeHandle& nh, bool is_left, const std::vector<double>& pose_xyzrpy, double gripper) {
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

    // Joint positions control
    void X7StateInterface::set_joint_pos_cmd(ros::NodeHandle& nh, bool is_left, const std::vector<double>& joint_angles) {
      std::string topic_name = is_left ? "/joint_control" : "/joint_control2";
      ros::Publisher pub = nh.advertise<arm_control::JointControl>(topic_name, 10);
      ros::Duration(1.0).sleep();

      arm_control::JointControl cmd;
      ROS_INFO_STREAM("Publishing joint_pos:");
      for (size_t i = 0; i < 8 && i < joint_angles.size(); ++i) {
      cmd.joint_pos[i] = static_cast<float>(joint_angles[i]);
      ROS_INFO_STREAM("joint_pos[" << i << "] = " << cmd.joint_pos[i]);
    }

      ros::Rate rate(10);
      for (int i = 0; i < 20; ++i) {
        pub.publish(cmd);
        rate.sleep();
      }
    }

    // pose service callback
    bool X7StateInterface::singlePoseCallback(arx_x7_controller::SinglePose::Request& req,
                                              arx_x7_controller::SinglePose::Response& res) {
        single_grasp_xyz = std::vector<double>(req.single_xyz.begin(), req.single_xyz.end());
        single_grasp_is_left = req.is_left;
        single_grasp_received = true;

        ROS_INFO_STREAM("Received " << (req.is_left ? "LEFT" : "RIGHT") << " single grasp pose");
        res.success = true;
        return true;
    }
    bool X7StateInterface::singleMovedPoseCallback(arx_x7_controller::SinglePose::Request& req,
                                              arx_x7_controller::SinglePose::Response& res) {
        single_grasp_xyz = std::vector<double>(req.single_xyz.begin(), req.single_xyz.end());
        single_grasp_is_left = req.is_left;
        single_moved_grasp_received = true;

        ROS_INFO_STREAM("Received " << (req.is_left ? "LEFT" : "RIGHT") << " single grasp pose");
        res.success = true;
        return true;
    }

    bool X7StateInterface::bimanualPoseCallback(arx_x7_controller::Pose3D::Request& req,
                                                arx_x7_controller::Pose3D::Response& res) {
        left_grasp_xyz = std::vector<double>(req.left_xyz.begin(), req.left_xyz.end());
        right_grasp_xyz = std::vector<double>(req.right_xyz.begin(), req.right_xyz.end());;
        bimanual_grasp_received  = true;
        ROS_INFO("Received BIMANUAL grasp poses");
        res.success = true;
        return true;
    }

    bool X7StateInterface::bimanualClosePoseCallback(arx_x7_controller::Pose3D::Request& req,
                                                arx_x7_controller::Pose3D::Response& res) {
        left_grasp_xyz = std::vector<double>(req.left_xyz.begin(), req.left_xyz.end());
        right_grasp_xyz = std::vector<double>(req.right_xyz.begin(), req.right_xyz.end());;
        bimanual_close_grasp_received  = true;
        ROS_INFO("Received BIMANUAL grasp poses");
        res.success = true;
        return true;
    }
    
    bool X7StateInterface::poseLoopCallback(arx_x7_controller::Pose_loop::Request& req, arx_x7_controller::Pose_loop::Response& res) {
        left_grasp_xyz = std::vector<double>(req.left_xyz.begin(), req.left_xyz.end());
        right_grasp_xyz = std::vector<double>(req.right_xyz.begin(), req.right_xyz.end());;
        num = req.num;
        bimanual_loop  = true;
        ROS_INFO("Received BIMANUAL grasp poses as loop");
        res.success = true;
        return true;
    }

      bool X7StateInterface::actionCallback(arx_x7_controller::ActionSuccess::Request& req, arx_x7_controller::ActionSuccess::Response& res){
        perception = req.perception;
        action_fin  = true;
        ROS_INFO("perception is finished");
        res.action = true;
        return true;
      }


    //pose getter
    std::pair<bool, std::vector<double>> X7StateInterface::get_single_grasp_pose() {
        if (!single_grasp_received)
            ROS_WARN("Single grasp pose not yet received.");
        return {single_grasp_is_left, single_grasp_xyz};
    }
    std::pair<bool, std::vector<double>> X7StateInterface::get_single_moved_grasp_pose() {
        if (!single_moved_grasp_received)
            ROS_WARN("Single moved grasp pose not yet received.");
        return {single_grasp_is_left, single_grasp_xyz};
    }
    std::pair<std::vector<double>, std::vector<double>> X7StateInterface::get_bimanual_grasp_pose() {
        if (!bimanual_grasp_received)
            ROS_WARN("Bimanual grasp pose not yet received.");
        return {left_grasp_xyz, right_grasp_xyz};
    }
    std::pair<std::vector<double>, std::vector<double>> X7StateInterface::get_bimanual_close_grasp_pose() {
        if (!bimanual_close_grasp_received)
            ROS_WARN("Bimanual close grasp pose not yet received.");
        return {left_grasp_xyz, right_grasp_xyz};
    }
    std::tuple<std::vector<double>, std::vector<double>, int> X7StateInterface::get_pose_loop() {
        if (!bimanual_loop)
            ROS_WARN("Bimanual close grasp pose loop not yet received.");
        return {left_grasp_xyz, right_grasp_xyz, num};
    }

     bool X7StateInterface::is_action_fin(){
        if (!action_fin)
            ROS_WARN("perception is finished");
        return perception;
    }
}// namespace arx::x7


//int main(int argc, char** argv) {
//  ros::init(argc, argv, "ros_control");
//  ros::NodeHandle nh;
//
//  arx::x7::X7StateInterface controller(nh);
//
//  bool use_vr = true;
//
//  if (use_vr) {
//    std::vector<double> left_pose = {0.1, 0.2, 0.2, 0.0, 0.0, 0.0};
//    std::vector<double> right_pose = {0.1, -0.2, 0.2, 0.0, 0.0, 0.0};
//    double left_gripper = 3.0;
//    double right_gripper = 3.0;
//
//    // 왼팔, 오른팔을 각각 스레드에서 실행
//    std::thread left_thread(&arx::x7::X7StateInterface::set_ee_pose_cmd, &controller, std::ref(nh), true, left_pose, left_gripper);
//    std::thread right_thread(&arx::x7::X7StateInterface::set_ee_pose_cmd, &controller, std::ref(nh), false, right_pose, right_gripper);
//
//    left_thread.join();
//    right_thread.join();
//
//  } else {
//    std::vector<double> joints = {-0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2};
//    controller.set_joint_pos_cmd(nh, true, joints);
//  }
//
//  return 0;
//}

