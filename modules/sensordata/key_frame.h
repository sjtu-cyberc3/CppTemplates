#pragma once

#include "pose.h"
#include <iostream>

class KeyFrame : public Pose {
 public:
  double       time  = 0.0;
  unsigned int index = 0;

  friend std::ostream& operator<<(std::ostream& os, const KeyFrame& data);
  friend std::istream& operator>>(std::istream& in, KeyFrame& data);
};
