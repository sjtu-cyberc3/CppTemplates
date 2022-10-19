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
#include <iostream>

#include <gtest/gtest.h>

#include "config/global_defination.h"
#include "submodule1.h"

TEST(submodule1, get_age) {
  modules::submodule1 stuA("Joel", 21);
  ASSERT_TRUE(stuA.get_age() == 16) << "age is not 16";
}

TEST(submodule1, get_name) {
  modules::submodule1 stuA("Joel", 21);
  ASSERT_EQ(stuA.get_name(), "Joel") << "name is not Joel";
}

TEST(submodule1, get_color) {
  modules::submodule1 stuA("Joel", 21);
  stuA.m_color = modules::submodule1::TEnum::BLUE;
  ASSERT_EQ(stuA.m_color, modules::submodule1::TEnum::BLUE) << "color is not BLUE";
}