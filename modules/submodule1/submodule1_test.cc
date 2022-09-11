/**
 * @file submodule1_test.cc
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-09-09
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <gtest/gtest.h>

#include "submodule1.h"

TEST(submodule1, get_age) {
  submodule1 myClass("Joel", 21);
  ASSERT_TRUE(myClass.get_age() == 16) << "age is not 16";
}

TEST(submodule1, get_name) {
  submodule1 myClass("Joel", 21);
  ASSERT_EQ(myClass.get_name(), "Joel") << "name is not Joel";
}
