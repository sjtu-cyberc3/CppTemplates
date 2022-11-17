#pragma once
#ifndef SENSORDATA_IMU_H
#define SENSORDATA_IMU_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iomanip>
#include <iostream>
#include <numeric>

namespace sensordata {

struct IMU {
  double             time;
  Eigen::Vector3d    acc;
  Eigen::Vector3d    gyr;
  Eigen::Quaterniond rot;
};

}  // namespace sensordata

inline std::ostream& operator<<(std::ostream& os, const sensordata::IMU& data) {
  os << std::setprecision(std::numeric_limits<double>::max_digits10) << std::fixed;
  os << data.time << " ";
  os << data.acc.x() << " " << data.acc.y() << " " << data.acc.z() << " ";
  os << data.gyr.x() << " " << data.gyr.y() << " " << data.gyr.z() << " ";
  os << data.rot.w() << " " << data.rot.x() << " " << data.rot.y() << " " << data.rot.z() << " ";
  return os;
}

inline std::istream& operator>>(std::istream& is, sensordata::IMU& data) {
  is >> data.time;
  is >> data.acc.x() >> data.acc.y() >> data.acc.z();
  is >> data.gyr.x() >> data.gyr.y() >> data.gyr.z();
  is >> data.rot.w() >> data.rot.x() >> data.rot.y() >> data.rot.z();
  return is;
}

#endif  // SENSORDATA_IMU_H
