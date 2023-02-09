#pragma once

#include "common/reflect.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iomanip>
#include <iostream>
#include <numeric>

namespace datatype {

struct IMU {
  double             time;
  Eigen::Vector3d    acc;
  Eigen::Vector3d    gyr;
  Eigen::Quaterniond rot;
};

}  // namespace datatype

STRUCT_INFO(datatype::IMU, , (time)(acc)(gyr)(rot))
