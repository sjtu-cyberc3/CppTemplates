#pragma once

#include <deque>
#include <iostream>

class GNSSData {
 public:
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

  friend std::ostream& operator<<(std::ostream& os, const GNSSData& data);
  friend std::istream& operator>>(std::istream& in, GNSSData& data);
};