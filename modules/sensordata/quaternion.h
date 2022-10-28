#pragma once

#include <Eigen/Dense>
#include <iostream>

class Quaternion {
 private:
  Eigen::Quaterniond q_;

 public:
  void set(double w, double x, double y, double z);
  void set(Eigen::Vector4d wxyz);
  void set(Eigen::Quaterniond wxyz);
  void set(double rx, double ry, double rz);
  void set(Eigen::Vector3d rpy);
  void set(Eigen::Matrix3d rotation_matrix);

  Eigen::Quaterniond quat();
  Eigen::Vector3d    rpy();
  Eigen::Matrix3d    matrix();

  friend std::ostream& operator<<(std::ostream& os, const Quaternion& data);
  friend std::istream& operator>>(std::istream& in, Quaternion& data);
};