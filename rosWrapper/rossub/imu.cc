#include "imu.h"

namespace roswrapper {
IMUSubscriber::IMUSubscriber(const ros::NodeHandle& nh, const std::string& topic_name, size_t buff_size) : nh_(nh) {
  subscriber_ = nh_.subscribe(topic_name, buff_size, &IMUSubscriber::msg_callback, this, ros::TransportHints().tcpNoDelay());
}

void IMUSubscriber::msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr) {
  std::lock_guard<std::mutex> lockGuard(mtx_);
  IMUData                     imu_data;

  msg_2_data(imu_msg_ptr, imu_data);

  new_imu_data_.push_back(imu_data);
}

void IMUSubscriber::msg_2_data(const sensor_msgs::ImuConstPtr& msgIn, IMUData& parse_data) {
  parse_data.time = msgIn->header.stamp.toSec();

  parse_data.linear_acceleration.set(msgIn->linear_acceleration.x, msgIn->linear_acceleration.y, msgIn->linear_acceleration.z);

  parse_data.angular_velocity.set(msgIn->angular_velocity.x, msgIn->angular_velocity.y, msgIn->angular_velocity.z);

  parse_data.rot.set(msgIn->orientation.w, msgIn->orientation.x, msgIn->orientation.y, msgIn->orientation.z);
}

void IMUSubscriber::ParseData(std::deque<IMUData>& imu_data_buff) {
  std::lock_guard<std::mutex> lockGuard(mtx_);
  if (new_imu_data_.size() > 0) {
    imu_data_buff.insert(imu_data_buff.end(), new_imu_data_.begin(), new_imu_data_.end());
    new_imu_data_.clear();
  }
}
}  // namespace roswrapper