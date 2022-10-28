#pragma once

#include <ros/ros.h>

#include "cyberc3_lio/cloud_info.h"
#include "modules/sensordata/cloud_data.h"

namespace roswrapper {
class CloudInfoPublisher {
 public:
  CloudInfoPublisher(const ros::NodeHandle& nh, const std::string& topic_name, const std::string& base_frame_id, int buff_size);
  CloudInfoPublisher() = default;
  void publish(cyberc3_lio::cloud_info& data, double time);

 private:
  ros::NodeHandle nh_;
  ros::Publisher  publisher_;
  std::string     frame_id_ = "";
};
}  // namespace roswrapper