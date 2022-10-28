#include "odometry.h"

namespace roswrapper {
OdometryPublisher::OdometryPublisher(
  const ros::NodeHandle& nh, const std::string& topic_name, const std::string& base_frame_id, const std::string& child_frame_id, int buff_size)
  : nh_(nh), frame_id_(base_frame_id), child_id_(child_frame_id) {
  publisher_ = nh_.advertise<nav_msgs::Odometry>(topic_name, buff_size);
}

void OdometryPublisher::publish(KeyFrame& key_frame) {
  if (publisher_.getNumSubscribers() != 0) {
    nav_msgs::Odometry laserOdometry;
    auto               rpy = key_frame.rot.rpy();

    laserOdometry.header.stamp = ros::Time().fromSec(key_frame.time);

    laserOdometry.header.frame_id = frame_id_;
    laserOdometry.child_frame_id  = child_id_;

    laserOdometry.pose.pose.position.x = key_frame.trans.x();
    laserOdometry.pose.pose.position.y = key_frame.trans.y();
    laserOdometry.pose.pose.position.z = key_frame.trans.z();

    laserOdometry.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rpy(0), rpy(1), rpy(2));

    publisher_.publish(laserOdometry);
  }
}
}  // namespace roswrapper