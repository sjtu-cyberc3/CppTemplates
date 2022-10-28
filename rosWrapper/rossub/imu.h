#pragma once

#include <glog/logging.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <deque>
#include <mutex>

#include "modules/sensordata/imu_data.h"

namespace roswrapper {
class IMUSubscriber {
 public:
  IMUSubscriber(const ros::NodeHandle& nh, const std::string& topic_name, size_t buff_size);
  IMUSubscriber() = default;
  void        ParseData(std::deque<IMUData>& deque_imu_data);
  static void msg_2_data(const sensor_msgs::ImuConstPtr& msgIn, IMUData& pasre_data);

 private:
  ros::NodeHandle     nh_;
  ros::Subscriber     subscriber_;
  std::mutex          mtx_;
  std::deque<IMUData> new_imu_data_;

 private:
  void msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);
};
}  // namespace roswrapper
