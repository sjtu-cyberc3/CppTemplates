#pragma once

#include <Eigen/Dense>
#include <deque>
#include <fstream>
#include <iostream>
#include <string>

#include "modules/common/file_manager.h"

namespace io {
class BasicIOType {
 public:
  BasicIOType(std::string path) {
    path_ = path;
  }

  void create_file() {
    FileManager::CreateFile(trajectory_ofs_, path_);
  };

 protected:
  std::string   path_;
  std::ofstream trajectory_ofs_;
};
}  // namespace io