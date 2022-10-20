#include "logger.hpp"
#define MODULE_NAME "GLOG_v1.2"

#include "config/global_defination.h"

int main(int argc, char** argv) {
  Logger logger(argc, argv, WORK_SPACE_PATH + "/build/");

  LOG(INFO) << "Hello Glog";
  LOG(WARNING) << "Hello Glog";
  AWARN << " Hello world!";
  return 0;
}