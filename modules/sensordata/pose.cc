#include "pose.h"

void Pose::set(const Eigen::Matrix4d& data) {
  trans.set(data(0, 3), data(1, 3), data(2, 3));

  Eigen::Matrix3d q = data.block<3, 3>(0, 0);
  rot.set(q);
}

void Pose::set(double x, double y, double z, double rx, double ry, double rz) {
  rot.set(rx, ry, rz);
  trans.set(x, y, z);
}

Eigen::Matrix4d Pose::matrix() {
  Eigen::Matrix4d pose   = Eigen::Matrix4d::Zero();
  pose.block<3, 3>(0, 0) = rot.matrix();

  pose(0, 3) = trans.x();
  pose(1, 3) = trans.y();
  pose(2, 3) = trans.z();
  pose(3, 3) = 1;
  return pose;
}

std::ostream& operator<<(std::ostream& os, const Pose& data) {
  os << data.trans << " " << data.rot << std::endl;
  return os;
}

std::istream& operator>>(std::istream& in, Pose& data) {
  in >> data.trans >> data.rot;
  return in;
}