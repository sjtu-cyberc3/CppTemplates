#pragma once
#include "basic_io.h"

namespace io {
class Time : public BasicIOType {
 public:
  Time(std::string path) : BasicIOType(path) {}

  bool save(const Eigen::Vector3d& time) {
    for (size_t i = 0; i < 3; ++i) {
      trajectory_ofs_ << time(i);
      if (i == 2) {
        trajectory_ofs_ << std::endl;
      } else {
        trajectory_ofs_ << " ";
      }
    }
    return true;
  }
};
}  // namespace io