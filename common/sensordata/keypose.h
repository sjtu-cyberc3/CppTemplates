#pragma once
#ifndef SENSORDATA_KEYPOSE_H
#define SENSORDATA_KEYPOSE_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iomanip>
#include <iostream>
#include <numeric>

namespace sensordata {

struct KeyPose : Pose {
  double       time  = 0.0;
  unsigned int index = 0;
};

}  // namespace sensordata

inline std::ostream& operator<<(std::ostream& os, const sensordata::KeyPose& data) {
  os << std::setprecision(std::numeric_limits<double>::max_digits10) << std::fixed;
  os << data.index << " " << data.time << " ";
  os << data.trans.x() << " " << data.trans.y() << " " << data.trans.z() << " ";
  os << data.rot.w() << " " << data.rot.x() << " " << data.rot.y() << " " << data.rot.z() << " ";
  return os;
}

inline std::istream& operator>>(std::istream& is, sensordata::KeyPose& data) {
  is >> data.index >> data.time;
  is >> data.trans.x() >> data.trans.y() >> data.trans.z();
  is >> data.rot.w() >> data.rot.x() >> data.rot.y() >> data.rot.z();
  return is;
}

#endif  // SENSORDATA_POSE_H
