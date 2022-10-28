#pragma once

#include <string>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include "modules/sensordata/key_frame.h"

namespace roswrapper {
class TFPublisher {
 public:
  TFPublisher(const std::string& base_frame_id, const std::string& child_frame_id);
  TFPublisher() = default;
  void publish(KeyFrame& key_frame);

 private:
  std::string frame_id_ = "";
  std::string child_id_ = "";

  tf::TransformBroadcaster br_;
};
}  // namespace roswrapper