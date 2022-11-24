#pragma once

#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

namespace converter {

template <typename P> void to_ros_msg(sensor_msgs::PointCloud2& msg, const pcl::PointCloud<P>& data) {
  pcl::toROSMsg(data, msg);
}

template <typename P> void from_ros_msg(const sensor_msgs::PointCloud2& msg, pcl::PointCloud<P>& data) {
  pcl::fromROSMsg(msg, data);
}

}  // namespace converter
