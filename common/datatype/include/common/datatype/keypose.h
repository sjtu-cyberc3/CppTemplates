#pragma once

#include "common/reflect.h"

#include "pose.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iomanip>
#include <iostream>
#include <numeric>

namespace datatype {

struct KeyPose : Pose {
  double       time  = 0.0;
  unsigned int index = 0;
};

}  // namespace datatype

STRUCT_INFO(datatype::KeyPose, (datatype::Pose), (time)(index))
