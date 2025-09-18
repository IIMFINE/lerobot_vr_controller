#ifndef LOG_H
#define LOG_H

#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <sys/syscall.h>
#include <thread>
#include <unistd.h>

// 获取当前时间戳的函数
inline std::string getCurrentTimestamp() {
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  auto ms = std::chrono::duration_cast<std::chrono::microseconds>(
                now.time_since_epoch()) %
            1000000;

  std::stringstream ss;
  ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
  ss << "." << std::setfill('0') << std::setw(6) << ms.count();
  return ss.str();
}

// 获取线程ID的函数
inline long getThreadId() { return syscall(SYS_gettid); }

// 获取文件名（不包含路径）的函数
inline const char *getFileName(const char *path) {
  const char *filename = strrchr(path, '/');
  return filename ? filename + 1 : path;
}

// LE_LOG_INFO 宏定义 - 支持流式输出
#define LE_LOG_INFO                                                            \
  std::cout << "[" << getCurrentTimestamp() << "]"                             \
            << "[info]"                                                        \
            << "[" << getThreadId() << "]"                                     \
            << "[" << getFileName(__FILE__) << ":" << __LINE__ << "] "

// LE_LOG_ERROR 宏定义 - 支持流式输出，使用 std::cerr
#define LE_LOG_ERROR                                                           \
  std::cerr << "[" << getCurrentTimestamp() << "]"                             \
            << "[error]"                                                       \
            << "[" << getThreadId() << "]"                                     \
            << "[" << getFileName(__FILE__) << ":" << __LINE__ << "] "

#endif // LOG_H
