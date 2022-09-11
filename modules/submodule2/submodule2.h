/**
 * @file submodule2.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-09-09
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once

#include <deque>
#include <fstream>
#include <iostream>
#include <string>

/**
 * @brief
 *
 */
class submodule2 {
 public:
  /**
   * @brief Construct a new my class object
   *
   * @param name
   * @param age
   */
  submodule2(const std::string& name, int age) {
    m_age  = age;
    m_name = name;
  }

 public:
  /**
   * @brief Get the age object
   *
   * @return int
   */
  int get_age() {
    return m_age;
  }

  void read(const std::string& path);
  /**
   * @brief Get the name object
   * @note asddddddasddddddasddddddasddddddasddddddasddddddasddddddasddddddasddddddasddddddasddddddasddddddasddddddasddddddasddddddasddddddasdddddd
   * @return std::string
   */
  std::string get_name() {
    return m_name;
  }

  std::deque<std::string> dddd_;

 private:
  int         m_age;   ///< Brief description after the member
  std::string m_name;  ///< submoduel 名字·
  int         m_age1;  ///< 面

  ///    颜色的枚举定义
  ///    该枚举定义了系统中需要用到的颜色\n
  ///    可以使用该枚举作为系统中颜色的标识
  enum TEnum {
    RED,    ///< 枚举，标识红色
    BLUE,   ///< 枚举，标志蓝色
    YELLOW  ///< 枚举，标志黄色.
  } enumVar;
};
