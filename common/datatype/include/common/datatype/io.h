#pragma once

#include "common/reflect/stream.h"

#include <fstream>
#include <numeric>
#include <sstream>
#include <string>

template <typename T> class SensorIO {
 public:
  SensorIO() = default;

  SensorIO(const std::string& filename, std::ios_base::openmode m = std::ios_base::in | std::ios_base::out) : fs(filename, m) {}

  bool open(const std::string& filename, std::ios_base::openmode m = std::ios_base::in | std::ios_base::out) {
    fs.open(filename, m);
    return fs.is_open();
  }

  bool is_open() const {
    return fs.is_open();
  }

  void close() {
    fs.close();
  }

  bool loadOnce(T& data) {
    if (!is_open())
      return false;
    std::string line;
    if (!std::getline(fs, line))
      return false;
    std::istringstream iss(line);
    iss >> data;
    if (iss.fail())
      return false;
    return true;
  }

  template <typename OutputIt> bool loadAll(OutputIt first) {
    if (!is_open())
      return false;
    std::string line;
    T           data;
    while (std::getline(fs, line)) {
      std::istringstream iss(line);
      iss >> data;
      if (iss.fail())
        return false;
      *(first++) = data;
    }
    return true;
  }

  bool saveOnce(const T& data) {
    if (!is_open())
      return false;
    fs << std::setprecision(std::numeric_limits<double>::max_digits10) << data << std::endl;
    if (fs.fail())
      return false;
    return true;
  }

  template <typename InputIt> bool saveAll(InputIt first, InputIt last) {
    if (!is_open())
      return false;
    for (InputIt it = first; it != last; it++) {
      fs << std::setprecision(std::numeric_limits<double>::max_digits10) << *it << std::endl;
      if (fs.fail())
        return false;
    }
    return true;
  }

 private:
  std::fstream fs;
};
