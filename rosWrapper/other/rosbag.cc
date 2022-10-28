#include "rosbag.h"

namespace roswrapper {
RosBag::RosBag(const std::string& bag_path, const std::string& gnss_topic, const std::string& imu_topic)
  : bag_path_(bag_path), gnss_topic_(gnss_topic), imu_topic_(imu_topic) {
  try {
    bag_.open(bag_path_, rosbag::bagmode::Read);
  } catch (rosbag::BagException) {
    std::cerr << "Error opening file " << bag_path << std::endl;
  }

  // view_.addQuery(bag_, rosbag::TypeQuery("sensor_msgs/PointCloud2"));
  view_.addQuery(bag_, rosbag::TypeQuery("sensor_msgs/Imu"));
  view_.addQuery(bag_, rosbag::TypeQuery("sensor_msgs/NavSatFix"));
}

void RosBag::read(std::deque<GNSSData>& queue_gnss, std::deque<IMUData>& queue_imu) {
  auto view_it = view_.begin();
  int  size    = view_.size();
  int  count   = 0;

  while (view_it != view_.end()) {
    std::string topic = view_it->getTopic();

    if (topic == gnss_topic_) {
      auto     Msg = view_it->instantiate<sensor_msgs::NavSatFix>();
      GNSSData data;
      roswrapper::GNSSSubscriber::msg_2_data(Msg, data);
      queue_gnss.push_back(data);
    } else if (topic == imu_topic_) {
      auto    Msg = view_it->instantiate<sensor_msgs::Imu>();
      IMUData data;
      roswrapper::IMUSubscriber::msg_2_data(Msg, data);
      queue_imu.push_back(data);
    }

    ++view_it;
    ++count;
    std::cout << count << "/" << size << std::endl;
  }
}

}  // namespace roswrapper
