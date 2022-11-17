#pragma once
#ifndef SENSORDATA_POSE_H
#define SENSORDATA_POSE_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iomanip>
#include <iostream>
#include <numeric>

namespace sensordata {

struct Pose {
  Eigen::Vector3d    trans;
  Eigen::Quaterniond rot;
};

}  // namespace sensordata

inline std::ostream& operator<<(std::ostream& os, const sensordata::Pose& data) {
  os << std::setprecision(std::numeric_limits<double>::max_digits10) << std::fixed;
  os << data.trans.x() << " " << data.trans.y() << " " << data.trans.z() << " ";
  os << data.rot.w() << " " << data.rot.x() << " " << data.rot.y() << " " << data.rot.z() << " ";
  return os;
}

inline std::istream& operator>>(std::istream& is, sensordata::Pose& data) {
  is >> data.trans.x() >> data.trans.y() >> data.trans.z();
  is >> data.rot.w() >> data.rot.x() >> data.rot.y() >> data.rot.z();
  return is;
}

#endif  // SENSORDATA_POSE_H
