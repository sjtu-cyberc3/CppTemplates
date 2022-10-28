#pragma once

#include <deque>
#include <mutex>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include "modules/sensordata/key_frame.h"

namespace roswrapper {
class OdometrySubscriber {
 public:
  OdometrySubscriber(const ros::NodeHandle& nh, const std::string& topic_name, size_t buff_size);
  OdometrySubscriber() = default;
  void ParseData(std::deque<KeyFrame>& deque_pose_data);
  void ParseCurrentData(KeyFrame& pose_data);

 private:
  ros::NodeHandle      nh_;
  ros::Subscriber      subscriber_;
  std::mutex           mtx_;
  std::deque<KeyFrame> new_pose_data_;

 private:
  void msg_callback(const nav_msgs::OdometryConstPtr& odom_msg_ptr);
};
}  // namespace roswrapper