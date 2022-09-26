#include "logger.hpp"
#define MODULE_NAME "GLOG_v1.2"

int main(int argc, char** argv) {
  Logger logger(argc, argv, "/Users/ztyu/test");

  LOG(INFO) << "Hello Glog";
  LOG(WARNING) << "Hello Glog";
  AWARN << " Hello world!";
  return 0;
}