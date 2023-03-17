#include "app/app_example.h"
#include <iostream>

AppExample::AppExample(const std::string& config_file) {
  // in this example, we do not need a config file.
  m_ = std::make_unique<ModuleExample>("Tom", 18);
}

void AppExample::feed_str(const std::string& str) {
  std::cout << __PRETTY_FUNCTION__ << " [str = " << str << "]" << std::endl;
  std::unique_lock lock(str_mtx_);
  str_queue_.emplace(str);
}

void AppExample::feed_int(int x) {
  std::cout << __PRETTY_FUNCTION__ << " [x = " << x << "]" << std::endl;
  std::unique_lock lock(int_mtx_);
  int_queue_.emplace(x);
}

void AppExample::feed_dcfg(int int_param, double double_param, const std::string& str_param) {
  std::cout << "dynamic param received!\n";
  std::cout << "int: " << int_param << " double: " << double_param << " str: " << str_param << std::endl;
}

void AppExample::process() {
  // check sensor data valid first
  // in this example, we need both string and int.
  if (str_queue_.empty() || int_queue_.empty()) {
    return;
  }
  // extract data
  std::string      str;
  std::unique_lock str_lock(str_mtx_);
  str = std::move(str_queue_.front());
  str_queue_.pop();
  str_lock.unlock();

  int              x;
  std::unique_lock int_lock(int_mtx_);
  x = std::move(int_queue_.front());
  int_queue_.pop();
  int_lock.unlock();

  // main process
  // in this example, we simply concatenate string and int.
  std::string rst = m_->do_work(str, x);

  // save result
  std::unique_lock rst_lock(rst_mtx_);
  rst_queue_.emplace(std::move(rst));
  rst_lock.unlock();
}

std::optional<std::string> AppExample::get_rst() {
  std::unique_lock rst_lock(rst_mtx_);
  if (rst_queue_.empty())
    return std::nullopt;
  std::string rst = std::move(rst_queue_.front());
  rst_queue_.pop();
  return std::move(rst);
}

std::string AppExample::service(const std::string& arg0, int arg1) {
  return arg0 + std::to_string(arg1);
}
