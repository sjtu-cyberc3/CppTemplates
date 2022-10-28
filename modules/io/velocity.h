#pragma once
#include "basic_io.h"

#include "modules/sensordata/velocity.h"

namespace io {
class VELOCITY : public BasicIOType {
 public:
  VELOCITY(std::string path) : BasicIOType(path) {}

  bool save(const Velocity& data) {
    trajectory_ofs_ << data;

    return true;
  }

  bool read(std::deque<Velocity>& deque) {
    std::ifstream infile(path_);
    assert(infile.is_open());

    deque.clear();

    std::string thisline;
    while (getline(infile, thisline)) {
      std::stringstream word(thisline);

      Velocity this_data;
      word >> this_data;

      deque.push_back(this_data);
    }
    infile.close();
    std::cout << deque.size() << std::endl;
  }
};
}  // namespace io