#include "key_frame.h"

std::ostream& operator<<(std::ostream& os, const KeyFrame& data) {
  os << std::fixed << data.time << " " << data.trans << " " << data.rot;
  return os;
}

std::istream& operator>>(std::istream& in, KeyFrame& data) {
  in >> data.time >> data.trans >> data.rot;
  return in;
}