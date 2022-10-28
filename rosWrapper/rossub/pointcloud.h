#pragma once

#include <deque>
#include <mutex>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "modules/sensordata/cloud_data.h"

namespace roswrapper {
class CloudSubscriber {
 public:
  CloudSubscriber(const ros::NodeHandle& nh, const std::string& topic_name, size_t buff_size);
  CloudSubscriber() = default;
  void ParseData(std::deque<CloudData>& deque_cloud_data);

 private:
  ros::NodeHandle       nh_;
  ros::Subscriber       subscriber_;
  std::mutex            mtx_;
  std::deque<CloudData> new_cloud_data_;

 private:
  void msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);
};
}  // namespace roswrapper