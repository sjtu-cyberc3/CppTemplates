#pragma once
#ifndef SENSORDATA_GNSS_H
#define SENSORDATA_GNSS_H

#include <iomanip>
#include <iostream>
#include <numeric>

namespace sensordata {

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

}  // namespace sensordata

inline std::ostream& operator<<(std::ostream& os, const sensordata::GNSS& data) {
  os << std::setprecision(std::numeric_limits<double>::max_digits10) << std::fixed;
  os << data.time << " ";
  os << data.longitude << " " << data.latitude << " " << data.altitude << " ";
  os << data.status << " " << data.service << " ";
  os << data.local_E << " " << data.local_N << " " << data.local_U << " ";
  return os;
}

inline std::istream& operator>>(std::istream& is, sensordata::GNSS& data) {
  is >> data.time;
  is >> data.longitude >> data.latitude >> data.altitude;
  is >> data.status >> data.service;
  is >> data.local_E >> data.local_N >> data.local_U;
  return is;
}

#endif  // SENSORDATA_GNSS_H