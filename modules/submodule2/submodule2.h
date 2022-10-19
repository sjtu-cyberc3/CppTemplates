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

#include <cassert>
#include <deque>
#include <fstream>
#include <iostream>
#include <string>

namespace modules {
/**
 * @brief submodule2用于读取文件
 *
 */
class submodule2 {
 public:
  /**
   * @brief Construct a new submodule2 object
   *
   */
  submodule2(){};
  /**
   * @brief 从文件读取信息
   *
   * @param path
   */
  void read(const std::string& path);

 public:
  /**
   * @brief 文件信息
   *
   */
  std::deque<std::string> info_;
};
}  // namespace modules