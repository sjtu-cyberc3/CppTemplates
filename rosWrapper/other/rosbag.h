#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>

#include "modules/sensordata/gnss_data.h"
#include "modules/sensordata/imu_data.h"

#include "rosWrapper/rossub/gnss.h"
#include "rosWrapper/rossub/imu.h"

namespace roswrapper {
class RosBag {
 public:
  RosBag(const std::string& bag_path, const std::string& gnss_topic, const std::string& imu_topic);

  void read(std::deque<GNSSData>& queue_gnss, std::deque<IMUData>& queue_imu);

 private:
  std::string bag_path_   = "";
  std::string gnss_topic_ = "";
  std::string imu_topic_  = "";

  rosbag::Bag  bag_;
  rosbag::View view_;
};
}  // namespace roswrapper
