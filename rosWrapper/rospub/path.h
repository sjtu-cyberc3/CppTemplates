#pragma once

#include <deque>
#include <string>

#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include "modules/sensordata/key_frame.h"

namespace roswrapper {
class PathPublisher {
 public:
  PathPublisher(
    const ros::NodeHandle& nh, const std::string& topic_name, const std::string& base_frame_id, const std::string& child_frame_id, int buff_size);
  PathPublisher() = default;
  void publish(std::deque<KeyFrame>& path);

 private:
  ros::NodeHandle nh_;
  ros::Publisher  publisher_;
  std::string     frame_id_ = "";
  std::string     child_id_ = "";
};
}  // namespace roswrapper