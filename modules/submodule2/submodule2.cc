/**
 * @file submodule2.cc
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-09-10
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "submodule2.h"

namespace modules {
void submodule2::read(const std::string& path) {
  std::ifstream infile(path);
  assert(infile.is_open());

  std::string thisline;
  while (getline(infile, thisline)) {
    std::cout << thisline << std::endl;

    info_.push_back(thisline);
  }
  infile.close();
  std::cout << info_.size() << std::endl;
}
}  // namespace modules