#pragma once

#include <deque>
#include <mutex>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

#include "cyberc3_lio/cloud_info.h"

namespace roswrapper {
class CloudInfoSubscriber {
 public:
  CloudInfoSubscriber(const ros::NodeHandle& nh, const std::string& topic_name, size_t buff_size);
  CloudInfoSubscriber() = default;
  void ParseData(std::deque<cyberc3_lio::cloud_info>& deque_cloud_data);

 private:
  ros::NodeHandle                     nh_;
  ros::Subscriber                     subscriber_;
  std::mutex                          mtx_;
  std::deque<cyberc3_lio::cloud_info> new_cloud_data_;

 private:
  void msg_callback(const cyberc3_lio::cloud_infoConstPtr& cloud_msg_ptr);
};
}  // namespace roswrapper