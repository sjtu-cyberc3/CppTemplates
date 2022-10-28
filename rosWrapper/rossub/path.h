#pragma once

#include <deque>
#include <mutex>
#include <string>

#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include "modules/sensordata/key_frame.h"

namespace roswrapper {
class PathSubscriber {
 public:
  PathSubscriber(const ros::NodeHandle& nh, const std::string& topic_name, size_t buff_size);
  PathSubscriber() = default;
  void        ParseData(std::deque<KeyFrame>& path);
  static void msg_2_data(const nav_msgs::PathConstPtr& msgIn, std::deque<KeyFrame>& pasre_data);

 private:
  ros::NodeHandle                  nh_;
  ros::Subscriber                  subscriber_;
  std::mutex                       mtx_;
  std::deque<std::deque<KeyFrame>> path_;

 private:
  void msg_callback(const nav_msgs::PathConstPtr& msgIn);
};
}  // namespace roswrapper
