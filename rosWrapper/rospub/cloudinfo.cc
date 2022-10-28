#include "cloudinfo.h"

namespace roswrapper {
CloudInfoPublisher::CloudInfoPublisher(const ros::NodeHandle& nh, const std::string& topic_name, const std::string& base_frame_id, int buff_size)
  : nh_(nh), frame_id_(base_frame_id) {
  publisher_ = nh_.advertise<cyberc3_lio::cloud_info>(topic_name, buff_size);
}

void CloudInfoPublisher::publish(cyberc3_lio::cloud_info& data, double time) {
  data.header.stamp    = ros::Time().fromSec(time);
  data.header.frame_id = frame_id_;
  if (publisher_.getNumSubscribers() != 0) {
    publisher_.publish(data);
  }
}
}  // namespace roswrapper