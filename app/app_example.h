#pragma once

#include "modules/example/example.h"

#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>

class AppExample {
 public:
  /****************************************/
  /*      construct from config file      */
  /****************************************/
  AppExample(const std::string& config_file);
  /****************************************/
  /*      sensor data feed functions      */
  /****************************************/
  void feed_str(const std::string& str);
  void feed_int(int x);

  /****************************************/
  /*        main process function         */
  /****************************************/
  void process();

  /****************************************/
  /*      result data get functions       */
  /****************************************/
  std::optional<std::string> get_rst();

  /****************************************/
  /*           service functions          */
  /****************************************/
  std::string service(const std::string &arg0, int arg1);

 private:
  /****************************************/
  /*      sensor data cache buffer        */
  /****************************************/
  std::queue<std::string> str_queue;
  std::queue<int>         int_queue;
  /****************************************/
  /*      sensor data cache mutex         */
  /****************************************/
  /* (optional) required only in a multi-threaded environment  */
  std::mutex str_mtx;
  std::mutex int_mtx;

  /****************************************/
  /*      result data cache buffer        */
  /****************************************/
  std::queue<std::string> rst_queue;
  /****************************************/
  /*      result data cache mutex         */
  /****************************************/
  /* (optional) required only in a multi-threaded environment  */
  std::mutex rst_mtx;

  /****************************************/
  /*      submodule  class                */
  /****************************************/
  std::unique_ptr<ModuleExample> m_;
};
