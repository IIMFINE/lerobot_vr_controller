#ifndef LOG_H
#define LOG_H

#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <sys/syscall.h>
#include <thread>
#include <type_traits>
#include <unistd.h>

using namespace std::chrono_literals;

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

// 辅助函数模板用于转换时间间隔参数
template <typename T>
constexpr std::chrono::milliseconds to_milliseconds(T &&duration) {
  if constexpr (std::is_arithmetic_v<std::decay_t<T>>) {
    // 如果是数字，假设单位是毫秒
    return std::chrono::milliseconds(duration);
  } else {
    // 如果是时间间隔类型，转换为毫秒
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration);
  }
}

// 内联函数模板用于检查是否应该输出日志（使用静态变量）
template <int LineNumber>
inline bool shouldLogWithThrottle(std::chrono::milliseconds interval) {
  static auto last_log_time = std::chrono::steady_clock::time_point::min();
  auto now = std::chrono::steady_clock::now();
  bool should_log = (last_log_time + interval <= now);
  if (should_log) {
    last_log_time = now;
  }
  return should_log;
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

// 字符串化宏
#define STRINGIFY(x) #x

// LE_LOG_INFO_T 宏定义 - 支持时间间隔控制的流式输出
#define LE_LOG_INFO_T(interval)                                                \
  if (shouldLogWithThrottle<__LINE__>(to_milliseconds(interval)))              \
  std::cout << "[" << getCurrentTimestamp() << "]"                             \
            << "[info]"                                                        \
            << "[" << STRINGIFY(interval) << "]"                               \
            << "[" << getThreadId() << "]"                                     \
            << "[" << getFileName(__FILE__) << ":" << __LINE__ << "] "

// LE_LOG_ERROR_T 宏定义 - 支持时间间隔控制的流式输出
#define LE_LOG_ERROR_T(interval)                                               \
  if (shouldLogWithThrottle<__LINE__>(to_milliseconds(interval)))              \
  std::cerr << "[" << getCurrentTimestamp() << "]"                             \
            << "[error]"                                                       \
            << "[" << STRINGIFY(interval) << "]"                               \
            << "[" << getThreadId() << "]"                                     \
            << "[" << getFileName(__FILE__) << ":" << __LINE__ << "] "

#endif // LOG_H
