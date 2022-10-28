#pragma once

#include "pose.h"

class LoopPose : public Pose {
 public:
  double       time = 0.0;
  unsigned int from = 0;
  unsigned int to   = 0;

  friend std::ostream& operator<<(std::ostream& os, const LoopPose& data);
  friend std::istream& operator>>(std::istream& in, LoopPose& data);
};
