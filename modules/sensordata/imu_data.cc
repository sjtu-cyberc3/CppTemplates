#include "imu_data.h"

Eigen::Vector3d IMUData::acc() {
  return linear_acceleration.xyz();
}

Eigen::Vector3d IMUData::vel() {
  return angular_velocity.xyz();
}

void IMUData::set_acc(Eigen::Vector3d data) {
  linear_acceleration.set(data);
}

void IMUData::set_vel(Eigen::Vector3d data) {
  angular_velocity.set(data);
}

std::ostream& operator<<(std::ostream& os, const IMUData& data) {
  os << std::fixed << data.time << " " << data.linear_acceleration << " " << data.angular_velocity << std::endl;
  return os;
}

std::istream& operator>>(std::istream& in, IMUData& this_data) {
  in >> this_data.time >> this_data.linear_acceleration >> this_data.angular_velocity;
  return in;
}