#include "quaternion.h"

void Quaternion::set(double w, double x, double y, double z) {
  Eigen::Quaterniond quaternion(w, x, y, z);
  q_ = quaternion;
}

void Quaternion::set(Eigen::Matrix3d rotation_matrix) {
  Eigen::Quaterniond quaternion(rotation_matrix);
  q_ = quaternion;
}

void Quaternion::set(Eigen::Vector4d wxyz) {
  set(wxyz(0), wxyz(1), wxyz(2), wxyz(3));
}

void Quaternion::set(Eigen::Quaterniond wxyz) {
  q_ = wxyz;
}

void Quaternion::set(double rx, double ry, double rz) {
  Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()));
  Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()));

  Eigen::Matrix3d rotation_matrix;
  rotation_matrix = yawAngle * pitchAngle * rollAngle;

  Eigen::Quaterniond quaternion(rotation_matrix);
  q_ = quaternion;
}

void Quaternion::set(Eigen::Vector3d rpy) {
  set(rpy(0), rpy(1), rpy(2));
}

Eigen::Quaterniond Quaternion::quat() {
  return q_;
}

Eigen::Vector3d Quaternion::rpy() {
  Eigen::Vector3d rpy, ypr;
  ypr    = q_.matrix().eulerAngles(2, 1, 0);
  rpy[0] = ypr[2];
  rpy[1] = ypr[1];
  rpy[2] = ypr[0];
  return rpy;
}

Eigen::Matrix3d Quaternion::matrix() {
  return q_.matrix();
}

std::ostream& operator<<(std::ostream& os, const Quaternion& data) {
  os << data.q_.x() << " " << data.q_.y() << " " << data.q_.z() << " " << data.q_.w() << std::endl;
  return os;
}

std::istream& operator>>(std::istream& in, Quaternion& data) {
  double w, x, y, z;
  in >> x >> y >> z >> w;
  data.set(w, x, y, z);
  return in;
}