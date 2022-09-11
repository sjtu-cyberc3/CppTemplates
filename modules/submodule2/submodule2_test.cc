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
#include <gtest/gtest.h>
#include <iostream>

#include "config/global_defination.h"

#include "submodule2.h"

TEST(test_submodule2, get_age) {
  submodule2 myClass("Joel", 21);
  ASSERT_TRUE(myClass.get_age() == 16) << "age is not 16";
}

TEST(test_submodule2, get_name) {
  submodule2 myClass("Joel", 21);
  ASSERT_EQ(myClass.get_name(), "Joel") << "name is not Joel";
}

TEST(test_submodule2, read) {
  submodule2  A("Tom", 18);
  std::string root_ = WORK_SPACE_PATH + "/modules/submodule2/data/info.txt";
  A.read(root_);

  ASSERT_EQ(A.dddd_[2], "000") << "string is not 000";
}
