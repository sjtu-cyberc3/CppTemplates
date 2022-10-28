#include "odometry.h"

namespace roswrapper {
OdometrySubscriber::OdometrySubscriber(const ros::NodeHandle& nh, const std::string& topic_name, size_t buff_size) : nh_(nh) {
  subscriber_ = nh_.subscribe(topic_name, buff_size, &OdometrySubscriber::msg_callback, this);
}

void OdometrySubscriber::msg_callback(const nav_msgs::OdometryConstPtr& odom_msg_ptr) {
  std::lock_guard<std::mutex> lockGuard(mtx_);

  KeyFrame pose_data;

  pose_data.time = odom_msg_ptr->header.stamp.toSec();

  double         roll, pitch, yaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(odom_msg_ptr->pose.pose.orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  pose_data.trans.set(odom_msg_ptr->pose.pose.position.x, odom_msg_ptr->pose.pose.position.y, odom_msg_ptr->pose.pose.position.z);
  pose_data.rot.set(roll, pitch, yaw);

  new_pose_data_.push_back(pose_data);
}

void OdometrySubscriber::ParseData(std::deque<KeyFrame>& pose_data_buff) {
  std::lock_guard<std::mutex> lockGuard(mtx_);
  if (new_pose_data_.size() > 0) {
    pose_data_buff.insert(pose_data_buff.end(), new_pose_data_.begin(), new_pose_data_.end());
    new_pose_data_.clear();
  }
}

void OdometrySubscriber::ParseCurrentData(KeyFrame& pose_data) {
  std::lock_guard<std::mutex> lockGuard(mtx_);
  if (new_pose_data_.size() > 0) {
    pose_data = new_pose_data_.back();
    new_pose_data_.clear();
  }
}
}  // namespace roswrapper
