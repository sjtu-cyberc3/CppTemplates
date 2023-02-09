#pragma once

#include "common/reflect.h"

#include <iomanip>
#include <iostream>
#include <limits>

namespace datatype {

struct GNSS {
  // Geodetic from GNSS
  double time      = 0.0;
  double longitude = 0.0;
  double latitude  = 0.0;
  double altitude  = 0.0;
  int    status    = 0;
  int    service   = 0;
  // Local cartesian coordinates.
  double local_E = 0.0;
  double local_N = 0.0;
  double local_U = 0.0;
};

}  // namespace datatype

STRUCT_INFO(datatype::GNSS, , (time)(longitude)(latitude)(altitude)(status)(service)(local_E)(local_N)(local_U))
