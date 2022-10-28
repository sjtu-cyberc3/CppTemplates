#include "cloudinfo.h"

namespace roswrapper {
CloudInfoSubscriber::CloudInfoSubscriber(const ros::NodeHandle& nh, const std::string& topic_name, size_t buff_size) : nh_(nh) {
  subscriber_ = nh_.subscribe(topic_name, buff_size, &CloudInfoSubscriber::msg_callback, this);
}

void CloudInfoSubscriber::msg_callback(const cyberc3_lio::cloud_infoConstPtr& cloud_msg_ptr) {
  std::lock_guard<std::mutex> lockGuard(mtx_);
  new_cloud_data_.push_back(*cloud_msg_ptr);
}

void CloudInfoSubscriber::ParseData(std::deque<cyberc3_lio::cloud_info>& cloud_data_buff) {
  std::lock_guard<std::mutex> lockGuard(mtx_);
  if (new_cloud_data_.size() > 0) {
    cloud_data_buff.insert(cloud_data_buff.end(), new_cloud_data_.begin(), new_cloud_data_.end());
    new_cloud_data_.clear();
  }
}
}  // namespace roswrapper