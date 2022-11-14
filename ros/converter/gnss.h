#pragma once
#ifndef ROSWRAPPER_CONVERTER_GNSS_H
#define ROSWRAPPER_CONVERTER_GNSS_H

#include "modules/sensordata/gnss.h"
#include <sensor_msgs/NavSatFix.h>

namespace converter {

inline void to_ros_msg(sensor_msgs::NavSatFix& msg, const sensordata::GNSS& data) {
  // Geodetic from GNSS
  msg.header.stamp.fromSec(data.time);
  msg.longitude      = data.longitude;
  msg.latitude       = data.latitude;
  msg.altitude       = data.altitude;
  msg.status.status  = data.status;
  msg.status.service = data.service;
  // Local cartesian coordinates.
}

inline void from_ros_msg(const sensor_msgs::NavSatFix& msg, sensordata::GNSS& data) {
  // Geodetic from GNSS
  data.time      = msg.header.stamp.toSec();
  data.longitude = msg.longitude;
  data.latitude  = msg.latitude;
  data.altitude  = msg.altitude;
  data.status    = msg.status.status;
  data.service   = msg.status.service;
  // Local cartesian coordinates.
}

}  // namespace converter

#endif  // ROSWRAPPER_CONVERTER_GNSS_H