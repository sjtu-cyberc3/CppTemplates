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

void submodule2::read(const std::string& path) {
  std::ifstream infile(path);
  assert(infile.is_open());

  std::string     thisline;
  std::deque<int> int_deque;
  while (getline(infile, thisline)) {
    std::cout << thisline << std::endl;

    // int this_data;
    // word >> this_data;

    dddd_.push_back(thisline);
  }
  infile.close();
  std::cout << int_deque.size() << std::endl;
}
