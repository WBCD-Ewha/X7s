#include <ros/ros.h>
#include <arm_control/PosCmd.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

class BodyTfPublisher {
public:
  BodyTfPublisher() {
    sub_ = nh_.subscribe("/body_information", 10, &BodyTfPublisher::callback, this);
  }

  void callback(const arm_control::PosCmd::ConstPtr& msg) {
    ros::Time stamp = ros::Time::now();

    // 1. base_link -> lift_link (z translation)
    tf::Transform base_to_lift;
    base_to_lift.setOrigin(tf::Vector3(0, 0, msg->height));
    base_to_lift.setRotation(tf::Quaternion::getIdentity());
    br_.sendTransform(tf::StampedTransform(base_to_lift, stamp, "base_link", "waist_link"));
//
//    // 2. lift_link -> waist_link (x translation)
//    tf::Transform lift_to_waist;
//    lift_to_waist.setOrigin(tf::Vector3(msg->tempFloatData[0], 0, 0));
//    lift_to_waist.setRotation(tf::Quaternion::getIdentity());
//    br_.sendTransform(tf::StampedTransform(lift_to_waist, stamp, "lift_link", "waist_link"));

    // 3. waist_link -> head_yaw_link (yaw rotation)
    tf::Transform waist_to_head_yaw;
    waist_to_head_yaw.setOrigin(tf::Vector3(0, 0, 0));
    waist_to_head_yaw.setRotation(tf::createQuaternionFromYaw(msg->head_yaw));
    br_.sendTransform(tf::StampedTransform(waist_to_head_yaw, stamp, "waist_link", "head_yaw_link"));

    // 4. head_yaw_link -> head_link (pitch rotation)
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

int main(int argc, char** argv) {
  ros::init(argc, argv, "body_tf_publisher");
  BodyTfPublisher publisher;
  ros::spin();
  return 0;
}
