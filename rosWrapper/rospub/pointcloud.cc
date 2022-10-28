#include "pointcloud.h"

namespace roswrapper {
CloudPublisher::CloudPublisher(const ros::NodeHandle& nh, const std::string& topic_name, const std::string& base_frame_id, int buff_size)
  : nh_(nh), frame_id_(base_frame_id) {
  publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, buff_size);
}

sensor_msgs::PointCloud2 CloudPublisher::publish(CloudData& data) {
  sensor_msgs::PointCloud2 tempCloud;
  pcl::toROSMsg(*data.ptr, tempCloud);
  tempCloud.header.stamp    = ros::Time().fromSec(data.time);
  tempCloud.header.frame_id = frame_id_;
  if (publisher_.getNumSubscribers() != 0)
    publisher_.publish(tempCloud);
  return tempCloud;
}

void CloudPublisher::publish(CloudData& data, double time, const Eigen::Matrix4f& pose) {
  sensor_msgs::PointCloud2 tempCloud;
  if (publisher_.getNumSubscribers() != 0) {
    pcl::transformPointCloud(*data.ptr, *data.ptr, pose);
    pcl::toROSMsg(*data.ptr, tempCloud);
    tempCloud.header.stamp    = ros::Time().fromSec(time);
    tempCloud.header.frame_id = frame_id_;
    publisher_.publish(tempCloud);
  }
}

}  // namespace roswrapper