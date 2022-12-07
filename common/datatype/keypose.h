#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iomanip>
#include <iostream>
#include <numeric>

#include "pose.h"

namespace datatype {

struct KeyPose : Pose {
  double       time  = 0.0;
  unsigned int index = 0;
};

}  // namespace datatype

inline std::ostream& operator<<(std::ostream& os, const datatype::KeyPose& data) {
  os << std::setprecision(std::numeric_limits<double>::max_digits10) << std::fixed;
  os << data.index << " " << data.time << " ";
  os << static_cast<const datatype::Pose&>(data) << " ";
  return os;
}

inline std::istream& operator>>(std::istream& is, datatype::KeyPose& data) {
  is >> data.index >> data.time;
  is >> static_cast<datatype::Pose&>(data);
  return is;
}
