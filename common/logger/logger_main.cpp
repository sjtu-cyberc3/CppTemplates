#include "logger.hpp"
#define MODULE_NAME "GLOG_v1.2"

#include <filesystem>

namespace fs = std::filesystem;

int main(int argc, char** argv) {
  Logger logger(argc, argv, fs::path(WORK_SPACE_PATH) / "build/");

  LOG(INFO) << "Hello Glog";
  LOG(WARNING) << "Hello Glog";
  AWARN << " Hello world!";
  return 0;
}