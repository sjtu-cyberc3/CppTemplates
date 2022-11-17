#include "logger.hpp"

Logger::Logger(int& argsize, char**& program, std::string&& path) {
  google::InitGoogleLogging(program[0]);

  FLAGS_log_dir                   = path;
  FLAGS_minloglevel               = 0;
  FLAGS_colorlogtostderr          = true;  // 设置输出到屏幕的日志显示对应颜色
  FLAGS_logbufsecs                = 0;     // 缓冲日志立即输出
  FLAGS_max_log_size              = 100;   // 最大日志大小为 100MB
  FLAGS_stop_logging_if_full_disk = true;  // 当磁盘被写满时，停止日志输出
  FLAGS_alsologtostderr           = true;  // 输出到屏幕的日志也输出到文件
}

Logger::~Logger() {
  google::ShutdownGoogleLogging();
  std::cout << "[LOGGER WARNING] Logger IS FINISHED!" << std::endl;
}
