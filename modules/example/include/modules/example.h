/**
 * @file ModuleExample.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-09-09
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once

#include <string>

namespace Tag {
///    颜色的枚举定义
///    该枚举定义了系统中需要用到的颜色\n
///    可以使用该枚举作为系统中颜色的标识
enum TEnum {
  RED,    ///< 枚举，标识红色
  BLUE,   ///< 枚举，标志蓝色
  YELLOW  ///< 枚举，标志黄色.
};
}  // namespace Tag

/**
 * @brief ModuleExample class
 *
 */
class ModuleExample {
 public:
  /**
   * @brief Construct a new ModuleExample object
   *
   * @param name 输入名字
   * @param age 输入年龄
   */
  ModuleExample(const std::string& name, int age) {
    age_  = age;
    name_ = name;
  }

  /**
   * @brief Set the age object
   *
   * @param age 输入年龄
   * @return void
   */
  void set_name(const std::string& name) {
    name_ = name;
  }

  /**
   * @brief Set the age object
   *
   * @param age 输入年龄
   * @return void
   */
  void set_age(int age) {
    age_ = age;
  }

  /**
   * @brief Get the age object
   *
   * @return int
   */
  int get_age() const {
    return age_;
  }

  /**
   * @brief Get the name object
   *
   * @return std::string
   */
  std::string get_name() {
    return name_;
  }

  std::string do_work(const std::string& str, const int num) {
    return str + "[" + std::to_string(num) + "], by " + name_;
  }

 private:
  int         age_;   ///< 年龄
  std::string name_;  ///< 姓名

 public:
  Tag::TEnum color_;  ///< 最喜爱颜色
};
