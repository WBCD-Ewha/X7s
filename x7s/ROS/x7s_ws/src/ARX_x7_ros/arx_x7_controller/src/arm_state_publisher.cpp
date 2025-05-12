#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <arm_control/JointInformation.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <map>
#include <ros/package.h>
#include "arx_x7_controller/x7_controller.hpp"

namespace arx::x7 {

class ArmTfPublisher {
public:
  ArmTfPublisher(ros::NodeHandle nh) : nh_(nh) {
    // Load parameters
    int arm_type = nh_.param("arm_type", 0);  // 0 = left, 1 = right

    std::string package_path = ros::package::getPath("arx_x7_controller");
    if (arm_type == 0) {
      urdf_path_ = package_path + "/X7Sleft1.urdf";
      joint_info_topic_ = "/joint_information";
    } else {
      urdf_path_ = package_path + "/x7sRIGHT.urdf";
      joint_info_topic_ = "/joint_information";
    }

    // Load URDF model
    urdf::Model model;
    if (!model.initFile(urdf_path_)) {
      ROS_ERROR("Failed to parse URDF from file: %s", urdf_path_.c_str());
      ros::shutdown();
    }

    if (!kdl_parser::treeFromUrdfModel(model, kdl_tree_)) {
      ROS_ERROR("Failed to construct KDL tree");
      ros::shutdown();
    }

    rsp_ = std::make_shared<robot_state_publisher::RobotStatePublisher>(kdl_tree_);

    // Subscribe to joint info topic
    joint_info_sub_ = nh_.subscribe(joint_info_topic_, 10, &ArmTfPublisher::jointInfoCallback, this);
  }

void jointInfoCallback(const arm_control::JointInformation::ConstPtr& msg) {
    ros::Time now = ros::Time::now();

    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.header.stamp = now;

    static const std::vector<std::string> joint_names = {
        "joint1", "joint2", "joint3", "joint4",
        "joint5", "joint6", "joint7", "joint8"
    };

    for (size_t i = 0; i < joint_names.size() && i < msg->joint_pos.size(); ++i) {
        joint_state_msg.name.push_back(joint_names[i]);
        joint_state_msg.position.push_back(msg->joint_pos[i]);
    }

    std::map<std::string, double> joint_positions;
    for (size_t i = 0; i < joint_state_msg.name.size(); ++i) {
        joint_positions[joint_state_msg.name[i]] = joint_state_msg.position[i];
    }

    rsp_->publishTransforms(joint_positions, now);
}


private:
  ros::NodeHandle nh_;
  ros::Subscriber joint_info_sub_;
  std::string urdf_path_;
  std::string joint_info_topic_;
  KDL::Tree kdl_tree_;
  std::shared_ptr<robot_state_publisher::RobotStatePublisher> rsp_;
};
}  // namespace arx::x7

int main(int argc, char** argv) {
  ros::init(argc, argv, "arm_state_publisher");
  ros::NodeHandle nh("~");
  arx::x7::ArmTfPublisher publisher(nh);
  ros::spin();
  return 0;
}
