#include "vector3.h"

void Vector3::set(double x, double y, double z) {
  xyz_[0] = x;
  xyz_[1] = y;
  xyz_[2] = z;
}

void Vector3::set_x(double x) {
  xyz_[0] = x;
}
void Vector3::set_y(double y) {
  xyz_[1] = y;
}
void Vector3::set_z(double z) {
  xyz_[2] = z;
}

void Vector3::set(Eigen::Vector3d xyz) {
  xyz_ = xyz;
}

Eigen::Vector3d Vector3::xyz() {
  return xyz_;
}

double Vector3::x() {
  return xyz_[0];
}

double Vector3::y() {
  return xyz_[1];
}

double Vector3::z() {
  return xyz_[2];
}

std::ostream& operator<<(std::ostream& os, const Vector3& data) {
  os << data.xyz_[0] << " " << data.xyz_[1] << " " << data.xyz_[2];
  return os;
}

std::istream& operator>>(std::istream& in, Vector3& this_data) {
  in >> this_data.xyz_[0] >> this_data.xyz_[1] >> this_data.xyz_[2];
  return in;
}