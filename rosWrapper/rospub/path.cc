#include "path.h"

namespace roswrapper {
PathPublisher::PathPublisher(
  const ros::NodeHandle& nh, const std::string& topic_name, const std::string& base_frame_id, const std::string& child_frame_id, int buff_size)
  : nh_(nh), frame_id_(base_frame_id), child_id_(child_frame_id) {
  publisher_ = nh_.advertise<nav_msgs::Path>(topic_name, buff_size);
}

void PathPublisher::publish(std::deque<KeyFrame>& path) {
  if (publisher_.getNumSubscribers() != 0) {
    nav_msgs::Path path1;
    path1.header.frame_id = "/map";

    for (size_t i = 0; i < path.size(); i++) {
      geometry_msgs::PoseStamped pose;

      path[i].set(path[i].matrix());

      pose.pose.position.x = path[i].trans.x();
      pose.pose.position.y = path[i].trans.y();
      pose.pose.position.z = path[i].trans.z();

      auto rpy              = path[i].rot.rpy();
      pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rpy(0), rpy(1), rpy(2));

      path1.poses.push_back(pose);
    }

    publisher_.publish(path1);
  }
}
}  // namespace roswrapper