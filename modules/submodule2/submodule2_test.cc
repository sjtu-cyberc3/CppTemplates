/**
 * @file submodule2_test.cc
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-09-09
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <iostream>

#include <gtest/gtest.h>

#include "config/global_defination.h"
#include "submodule2.h"

TEST(test_submodule2, read) {
  modules::submodule2 A;
  std::string         root_ = WORK_SPACE_PATH + "/modules/submodule2/data/info.txt";
  A.read(root_);

  ASSERT_EQ(A.info_[2], "000") << "string is not 000";
}
