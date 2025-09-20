#include "log.h"
#include <chrono>
#include <thread>

int main() {
  std::cout << "开始测试时间间隔日志功能..." << std::endl;

  // 运行10秒钟的测试
  auto start_time = std::chrono::steady_clock::now();
  auto duration = std::chrono::seconds(10);

  while (std::chrono::steady_clock::now() - start_time < duration) {
    // 每1秒输出一次，使用简单的数字参数
    LE_LOG_INFO_T(1000ms) << "Info: Hello world every 1 second" << std::endl;
    LE_LOG_ERROR_T(500ms) << "Error: Hello world every 500ms" << std::endl;

    // 测试新支持的写法
    LE_LOG_ERROR_T(100ms) << "Error: New format every 100ms" << std::endl;

    // 另一个位置的日志，使用不同的间隔
    LE_LOG_INFO_T(2000) << "Info: Different location, every 2 seconds"
                        << std::endl;

    // 稍微休眠一下，避免CPU占用过高
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  std::cout << "测试结束" << std::endl;
  return 0;
}
