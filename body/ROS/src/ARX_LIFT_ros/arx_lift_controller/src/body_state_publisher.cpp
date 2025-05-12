#include <ros/ros.h>
#include <arx_lift_src/lift_head_control_loop.h>
#include <arm_control/PosCmd.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>


namespace arx {

class BodyTfPublisher {
public:
  BodyTfPublisher(ros::NodeHandle nh) : nh_(nh) {
    std::string topic_name = nh_.param<std::string>("body_info_topic", "/body_information");
    sub_ = nh_.subscribe(topic_name, 10, &BodyTfPublisher::callback, this);
    ROS_INFO("BodyTfPublisher initialized. Subscribed to: %s", topic_name.c_str());
  }

  void callback(const arm_control::PosCmd::ConstPtr& msg) {
    ros::Time stamp = ros::Time::now();

    // base_link → waist_link (z translation, lift height)
    tf::Transform base_to_waist;
    base_to_waist.setOrigin(tf::Vector3(0, 0, msg->height));
    base_to_waist.setRotation(tf::Quaternion::getIdentity());
    br_.sendTransform(tf::StampedTransform(base_to_waist, stamp, "base_link", "waist_link"));

    // waist_link → head_yaw_link (yaw)
    tf::Transform waist_to_head_yaw;
    waist_to_head_yaw.setOrigin(tf::Vector3(0, 0, 0));
    waist_to_head_yaw.setRotation(tf::createQuaternionFromYaw(msg->head_yaw));
    br_.sendTransform(tf::StampedTransform(waist_to_head_yaw, stamp, "waist_link", "head_yaw_link"));

    // head_yaw_link → head_link (pitch)
    tf::Transform head_yaw_to_head;
    head_yaw_to_head.setOrigin(tf::Vector3(0, 0, 0));
    head_yaw_to_head.setRotation(tf::createQuaternionFromRPY(0, msg->head_pit, 0));
    br_.sendTransform(tf::StampedTransform(head_yaw_to_head, stamp, "head_yaw_link", "head_link"));
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  tf::TransformBroadcaster br_;
};

} // namespace arx::body

int main(int argc, char** argv) {
  ros::init(argc, argv, "body_state_publisher");
  ros::NodeHandle nh("~");
  arx::BodyTfPublisher publisher(nh);
  ros::spin();
  return 0;
}
