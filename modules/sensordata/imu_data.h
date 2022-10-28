#pragma once

#include <Eigen/Dense>
#include <iostream>

#include "quaternion.h"
#include "vector3.h"

class IMUData {
 public:
  double     time = 0.0;
  Vector3    linear_acceleration;
  Vector3    angular_velocity;
  Quaternion rot;

 public:
  Eigen::Vector3d acc();
  Eigen::Vector3d vel();

  void set_acc(Eigen::Vector3d data);
  void set_vel(Eigen::Vector3d data);

  friend std::ostream& operator<<(std::ostream& os, const IMUData& data);
  friend std::istream& operator>>(std::istream& in, IMUData& data);
};
