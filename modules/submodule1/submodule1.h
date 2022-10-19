/**
 * @file submodule1.h
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

namespace modules {
/**
 * @brief 子模块1: 姓名、年龄管理类
 *
 */
class submodule1 {
 public:
  /**
   * @brief Construct a new submodule1 object
   *
   * @param name 输入名字
   * @param age 输入年龄
   */
  submodule1(const std::string& name, int age) {
    m_age  = age;
    m_name = name;
  }

  /**
   * @brief Set the age object
   *
   * @param age 输入年龄
   * @return int
   */
  int set_age(int age) {
    m_age = age;
  }

  /**
   * @brief Get the age object
   *
   * @return int
   */
  int get_age() {
    return m_age;
  }

  /**
   * @brief Get the name object
   *
   * @return std::string
   */
  std::string get_name() {
    return m_name;
  }

 private:
  int         m_age;   ///< 年龄
  std::string m_name;  ///< 姓名

 public:
  ///    颜色的枚举定义
  ///    该枚举定义了系统中需要用到的颜色\n
  ///    可以使用该枚举作为系统中颜色的标识
  enum TEnum {
    RED,    ///< 枚举，标识红色
    BLUE,   ///< 枚举，标志蓝色
    YELLOW  ///< 枚举，标志黄色.
  };
  TEnum m_color;  ///< 颜色
};
}  // namespace modules