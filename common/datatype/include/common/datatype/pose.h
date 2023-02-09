#pragma once

#include "common/reflect.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iomanip>
#include <iostream>
#include <numeric>

namespace datatype {

struct Pose {
  Eigen::Vector3d    trans;
  Eigen::Quaterniond rot;
};

}  // namespace datatype

STRUCT_INFO(datatype::Pose, , (trans)(rot))
