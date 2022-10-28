#include "path.h"

namespace roswrapper {
PathSubscriber::PathSubscriber(const ros::NodeHandle& nh, const std::string& topic_name, size_t buff_size) : nh_(nh) {
  subscriber_ = nh_.subscribe(topic_name, buff_size, &PathSubscriber::msg_callback, this, ros::TransportHints().tcpNoDelay());
}

void PathSubscriber::msg_callback(const nav_msgs::PathConstPtr& msgIn) {
  std::lock_guard<std::mutex> lockGuard(mtx_);

  std::deque<KeyFrame> path;

  msg_2_data(msgIn, path);

  path_.push_back(path);
}

void PathSubscriber::msg_2_data(const nav_msgs::PathConstPtr& msgIn, std::deque<KeyFrame>& parse_data) {
  for (size_t i = 0; i < msgIn->poses.size(); i++) {
    KeyFrame                   thiskey;
    geometry_msgs::PoseStamped pose = msgIn->poses[i];

    double         roll, pitch, yaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(pose.pose.orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    thiskey.trans.set(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    thiskey.rot.set(roll, pitch, yaw);
    thiskey.index = i;

    parse_data.push_back(thiskey);
  }
}

void PathSubscriber::ParseData(std::deque<KeyFrame>& path) {
  std::lock_guard<std::mutex> lockGuard(mtx_);
  if (path_.size() > 0) {
    path = path_.back();
    // path.insert(path.end(), path_.begin(), path_.end());
    path_.clear();
  }
}
}  // namespace roswrapper