#pragma once

#include <string>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include "modules/sensordata/key_frame.h"

namespace roswrapper {
class OdometryPublisher {
 public:
  OdometryPublisher(
    const ros::NodeHandle& nh, const std::string& topic_name, const std::string& base_frame_id, const std::string& child_frame_id, int buff_size);
  OdometryPublisher() = default;
  void publish(KeyFrame& key_frame);

 private:
  ros::NodeHandle nh_;
  ros::Publisher  publisher_;
  std::string     frame_id_ = "";
  std::string     child_id_ = "";
};
}  // namespace roswrapper