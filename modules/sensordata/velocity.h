#pragma once

#include "vector3.h"
#include <iostream>

class Velocity {
 public:
  double  time = 0.0;
  Vector3 vel;

  friend std::ostream& operator<<(std::ostream& os, const Velocity& data);
  friend std::istream& operator>>(std::istream& in, Velocity& data);
};
