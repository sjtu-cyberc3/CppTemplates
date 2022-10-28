#include "loop_pose.h"

std::ostream& operator<<(std::ostream& os, const LoopPose& data) {
  os << std::fixed << data.time << " " << data.trans << " " << data.rot << std::endl;
  return os;
}

std::istream& operator>>(std::istream& in, LoopPose& data) {
  in >> data.time >> data.trans >> data.rot;
  return in;
}