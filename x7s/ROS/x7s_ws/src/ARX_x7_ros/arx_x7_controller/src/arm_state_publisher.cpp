#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <arm_control/JointInformation.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <map>
#include <ros/package.h>

class ArmTfPublisher {
public:
  ArmTfPublisher(const std::string& urdf_path, const std::string& joint_info_topic)
      : nh_("~"), joint_info_topic_(joint_info_topic) {
    // URDF 모델 로딩
    urdf::Model model;
    if (!model.initFile(urdf_path)) {
      ROS_ERROR("Failed to parse URDF file");
      return;
    }

    // KDL 트리 생성
    if (!kdl_parser::treeFromUrdfModel(model, kdl_tree_)) {
      ROS_ERROR("Failed to construct KDL tree");
      return;
    }

    // RobotStatePublisher 초기화
    rsp_ = std::make_shared<robot_state_publisher::RobotStatePublisher>(kdl_tree_);

    // 조인트 상태 구독
    joint_info_sub_ = nh_.subscribe(joint_info_topic_, 10, &ArmTfPublisher::jointInfoCallback, this);
  }

  void jointInfoCallback(const arm_control::JointInformation::ConstPtr& msg) {
    // JointState 메시지 생성
    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.header.stamp = ros::Time::now();

    // 조인트 이름과 위치 설정
    for (size_t i = 0; i < msg->joint_pos.size(); ++i) {
      joint_state_msg.name.push_back("joint_" + std::to_string(i + 1));
      joint_state_msg.position.push_back(msg->joint_pos[i]);
    }

    // TF 퍼블리시
    std::map<std::string, double> joint_positions;
    for (size_t i = 0; i < joint_state_msg.name.size(); ++i) {
      joint_positions[joint_state_msg.name[i]] = joint_state_msg.position[i];
    }
    rsp_->publishTransforms(joint_positions, joint_state_msg.header.stamp);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber joint_info_sub_;
  std::string joint_info_topic_;
  KDL::Tree kdl_tree_;
  std::shared_ptr<robot_state_publisher::RobotStatePublisher> rsp_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "dual_arm_tf_publisher");

  std::string package_path = ros::package::getPath("arx_x7_controller");
  std::string urdf_path;

  // 왼팔과 오른팔에 대한 URDF 경로 및 토픽 설정
  std::string left_arm_urdf = package_path + "/X7Sleft1.urdf";
  std::string right_arm_urdf = package_path + "/x7sRIGHT.urdf";
  std::string left_joint_topic = "/joint_control/joint_information";
  std::string right_joint_topic = "/joint_control2/joint_information";

  // 왼팔과 오른팔 TF 퍼블리셔 생성
  ArmTfPublisher left_arm_publisher(left_arm_urdf, left_joint_topic);
  ArmTfPublisher right_arm_publisher(right_arm_urdf, right_joint_topic);

  ros::spin();
  return 0;
}
