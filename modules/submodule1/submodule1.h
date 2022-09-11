/**
 * @file submodule1.h
 * @author ztyu@1231231
 * @brief submodule开发文件
 * @version 0.1
 * @date 2022-09-09
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once

#include <list>
#include <string>

/**
 * @brief 本类用于实现栋房屋e分的功能
 *
 */
class submodule1 {
  /**
   * @brief
   *
   */
 public:
  /**
   * @brief Construct a new submodule1 object
   *
   * @param name
   * @param age
   */
  submodule1(const std::string& name, int age) {
    m_age  = age;
    m_name = name;
  }
  /**
   * @brief 测试一个简单的demo
   *
   * @param name 输入名字
   * @param age 输年龄
   * @param abs 输入abs
   * @return int 状态码
   */
  int testfunction(const std::string& name, int age, std::list<int> abs) {
    m_age  = age;
    m_name = name;
    abs.push_back(age);
    return 1;
  }

 public:
  /**
   * @brief Get the age object
   *
   * @return int 三扽东方
   */
  int get_age() {
    return m_age;
  }

  /**
   * @brief Get the name object
   *
   * @return std::string sdfsdfs
   */
  std::string get_name() {
    return m_name;
  }

 private:
  int         m_age;
  std::string m_name;
};
