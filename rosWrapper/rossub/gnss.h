#pragma once

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include <deque>
#include <mutex>

#include "modules/sensordata/gnss_data.h"

namespace roswrapper {
class GNSSSubscriber {
 public:
  GNSSSubscriber(const ros::NodeHandle& nh, const std::string& topic_name, size_t buff_size);
  GNSSSubscriber() = default;
  void        ParseData(std::deque<GNSSData>& deque_gnss_data);
  static void msg_2_data(const sensor_msgs::NavSatFixConstPtr& msgIn, GNSSData& pasre_data);

 private:
  ros::NodeHandle      nh_;
  ros::Subscriber      subscriber_;
  std::mutex           mtx_;
  std::deque<GNSSData> new_gnss_data_;

 private:
  void msg_callback(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr);
};
}  // namespace roswrapper
