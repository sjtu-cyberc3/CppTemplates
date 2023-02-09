/**
 * @file ModuleExample_test.cc
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-17
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <iostream>

#include <gtest/gtest.h>

#include "modules/example.h"

TEST(example, get_age) {
  ModuleExample stu_a("Joel", 21);
  stu_a.color_ = Tag::TEnum::BLUE;
  ASSERT_TRUE(stu_a.get_age() == 21) << "age is not 21";
  ASSERT_EQ(stu_a.get_name(), "Joel") << "name is not Joel";
  ASSERT_EQ(stu_a.color_, Tag::TEnum::BLUE) << "color is not BLUE";
}