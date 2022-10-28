#pragma once

#include <Eigen/Dense>
#include <iostream>

#include "quaternion.h"
#include "vector3.h"

class Pose {
 public:
  Vector3    trans;
  Quaternion rot;

 public:
  void            set(double x, double y, double z, double rx, double ry, double rz);
  void            set(const Eigen::Matrix4d& data);
  Eigen::Matrix4d matrix();

  friend std::ostream& operator<<(std::ostream& os, const Pose& data);
  friend std::istream& operator>>(std::istream& in, Pose& data);
};