#pragma once

#include <Eigen/Dense>
#include <iostream>

class Vector3 {
 public:
  Eigen::Vector3d xyz_ = Eigen::Vector3d::Identity();

 public:
  void set(double x, double y, double z);
  void set(Eigen::Vector3d xyz);
  void set_x(double x);
  void set_y(double y);
  void set_z(double z);

  Eigen::Vector3d xyz();
  double          x();
  double          y();
  double          z();

  friend std::ostream& operator<<(std::ostream& os, const Vector3& data);
  friend std::istream& operator>>(std::istream& in, Vector3& data);
};