#include "velocity.h"

std::ostream& operator<<(std::ostream& os, const Velocity& data) {
  os << std::fixed << data.time << " " << data.vel;
  return os;
}

std::istream& operator>>(std::istream& in, Velocity& data) {
  in >> data.time >> data.vel;
  return in;
}