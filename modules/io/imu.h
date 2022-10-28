#pragma once
#include "basic_io.h"

#include "modules/sensordata/imu_data.h"

namespace io {
class IMU : public BasicIOType {
 public:
  IMU(std::string path) : BasicIOType(path) {}

  bool save(const IMUData& data) {
    trajectory_ofs_ << data;

    return true;
  }

  bool read(std::deque<IMUData>& deque) {
    std::ifstream infile(path_);
    assert(infile.is_open());

    deque.clear();

    std::string thisline;
    while (getline(infile, thisline)) {
      std::stringstream word(thisline);

      IMUData this_data;
      word >> this_data;

      deque.push_back(this_data);
    }
    infile.close();
    std::cout << deque.size() << std::endl;
  }
};
}  // namespace io