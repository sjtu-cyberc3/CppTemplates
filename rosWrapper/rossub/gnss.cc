#include "gnss.h"

namespace roswrapper {
GNSSSubscriber::GNSSSubscriber(const ros::NodeHandle& nh, const std::string& topic_name, size_t buff_size) : nh_(nh) {
  subscriber_ = nh_.subscribe(topic_name, buff_size, &GNSSSubscriber::msg_callback, this, ros::TransportHints().tcpNoDelay());
}

void GNSSSubscriber::msg_callback(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr) {
  std::lock_guard<std::mutex> lockGuard(mtx_);
  GNSSData                    gnss_data;

  msg_2_data(nav_sat_fix_ptr, gnss_data);

  new_gnss_data_.push_back(gnss_data);
}

void GNSSSubscriber::msg_2_data(const sensor_msgs::NavSatFixConstPtr& msgIn, GNSSData& pasre_data) {
  // Geodetic from GNSS
  pasre_data.time      = msgIn->header.stamp.toSec();
  pasre_data.longitude = msgIn->longitude;
  pasre_data.latitude  = msgIn->latitude;
  pasre_data.altitude  = msgIn->altitude;
  pasre_data.status    = msgIn->status.status;
  pasre_data.service   = msgIn->status.service;
  // Local cartesian coordinates.
  pasre_data.local_E = 0.0;
  pasre_data.local_N = 0.0;
  pasre_data.local_U = 0.0;
}

void GNSSSubscriber::ParseData(std::deque<GNSSData>& gnss_data_buff) {
  std::lock_guard<std::mutex> lockGuard(mtx_);
  if (new_gnss_data_.size() > 0) {
    gnss_data_buff.insert(gnss_data_buff.end(), new_gnss_data_.begin(), new_gnss_data_.end());
    new_gnss_data_.clear();
  }
}
}  // namespace roswrapper