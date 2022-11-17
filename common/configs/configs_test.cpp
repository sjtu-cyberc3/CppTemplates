#include <iostream>

#include <gtest/gtest.h>

#include "config/global_defination.h"
#include "configs.hpp"

TEST(configs, loadsingle) {
  std::string config_file_path = WORK_SPACE_PATH + "/common/configs_example.yaml";

  Configs cfg;
  ConfigDef(cfg, int, param1);
  ConfigDef(cfg, float, param2);
  ConfigDef(cfg, double, param3);
  ConfigDef(cfg, bool, param4);
  ConfigDef(cfg, std::string, param5);
  cfg.Open(config_file_path);
  cfg.LoadOnce();

  ASSERT_EQ(param1, 1) << "param1 is not 1";
  ASSERT_EQ(param2, 2.0) << "param2 is not 2.0";
  ASSERT_EQ(param3, 3.0) << "param3 is not 3.0";
  ASSERT_EQ(param4, true) << "param4 is not true";
  ASSERT_EQ(param5, "hello, world!") << "param5 is not hello, world!";
}