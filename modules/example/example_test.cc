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

#include "config/global_defination.h"
#include "example.h"

TEST(example, get_age) {
  ModuleExample stuA("Joel", 21);
  stuA.color_ = Tag::TEnum::BLUE;
  ASSERT_TRUE(stuA.get_age() == 21) << "age is not 21";
  ASSERT_EQ(stuA.get_name(), "Joel") << "name is not Joel";
  ASSERT_EQ(stuA.color_, Tag::TEnum::BLUE) << "color is not BLUE";
}