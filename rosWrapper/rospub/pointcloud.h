#pragma once

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "modules/sensordata/cloud_data.h"

namespace roswrapper {
class CloudPublisher {
 public:
  CloudPublisher(const ros::NodeHandle& nh, const std::string& topic_name, const std::string& base_frame_id, int buff_size);
  CloudPublisher() = default;
  sensor_msgs::PointCloud2 publish(CloudData& data);
  void                     publish(CloudData& data, double time, const Eigen::Matrix4f& pose = Eigen::Matrix4f::Identity());

 private:
  ros::NodeHandle nh_;
  ros::Publisher  publisher_;
  std::string     frame_id_ = "";
};
}  // namespace roswrapper