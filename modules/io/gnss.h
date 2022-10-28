#pragma once
#include "basic_io.h"

#include "modules/sensordata/gnss_data.h"

namespace io {
class GNSS : public BasicIOType {
 public:
  GNSS(std::string path) : BasicIOType(path) {}

  bool save(const GNSSData& data) {
    trajectory_ofs_ << data;

    return true;
  }

  bool read(std::deque<GNSSData>& deque) {
    std::ifstream infile(path_);
    assert(infile.is_open());

    deque.clear();

    std::string thisline;
    while (getline(infile, thisline)) {
      std::stringstream word(thisline);

      GNSSData this_data;
      word >> this_data;

      deque.push_back(this_data);
    }
    infile.close();
    std::cout << deque.size() << std::endl;
  }
};
}  // namespace io