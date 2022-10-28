#pragma once
#include "basic_io.h"

#include "modules/sensordata/loop_pose.h"

namespace io {
class LoopInfo : public BasicIOType {
 public:
  LoopInfo(std::string path) : BasicIOType(path) {}

  bool save(std::deque<LoopPose>& loopdeque) {
    for (size_t i = 0; i < loopdeque.size(); ++i) {
      auto wxyz = loopdeque[i].rot.quat();
      trajectory_ofs_ << std::fixed << loopdeque[i].from << " " << loopdeque[i].to << " " << loopdeque[i].trans.x() << " " << loopdeque[i].trans.y()
                      << " " << loopdeque[i].trans.z() << " " << wxyz.x() << " " << wxyz.y() << " " << wxyz.z() << " " << wxyz.w() << std::endl;
    }
    return true;
  }

  bool read(std::deque<LoopPose>& loopdeque) {
    std::ifstream infile;
    infile.open(path_);
    assert(infile.is_open());

    std::string temp;

    double num[8];

    int line = 0;
    while (getline(infile, temp)) {
      int               i = 0;
      std::stringstream word(temp);
      std::string       s;
      while (word >> s) {
        double a = atof(s.c_str());
        num[i]   = a;
        i++;
      }
      LoopPose newkey;
      newkey.from = num[0];
      newkey.to   = num[1];
      newkey.trans.set(num[2], num[3], num[4]);
      newkey.rot.set(num[8], num[5], num[6], num[7]);

      loopdeque.push_back(newkey);
      line++;
    }
    infile.close();
    std::cout << loopdeque.size() << std::endl;
  }
};
}  // namespace io