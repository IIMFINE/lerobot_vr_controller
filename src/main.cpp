#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vr_controller.h"

int main(int argc, char **argv) {
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Create node
  auto node = std::make_shared<rclcpp::Node>("vr_tf_receiver_node");

  // Create VrTfReceiver instance
  auto vr_tf_receiver =
      std::make_unique<lerobot_vr_controller::VrTfReceiver>(node);

  // Initialize YAML configuration (you may want to make this configurable via
  // parameter)
  std::string yaml_config_path = "config/vr_to_arm.yaml"; // Default path

  // Try to get config path from parameter
  node->declare_parameter<std::string>("config_file", yaml_config_path);
  node->get_parameter("config_file", yaml_config_path);

  printf("pan using config file: %s\n", yaml_config_path.c_str());

  if (!vr_tf_receiver->InitializeYamlConfig(yaml_config_path)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize YAML configuration");
    rclcpp::shutdown();
    return -1;
  }

  // Start VR TF receiver
  vr_tf_receiver->Start();

  // Create multi-threaded executor
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  RCLCPP_INFO(node->get_logger(), "VR TF Receiver node started successfully");

  // Spin the executor in a separate thread
  std::thread spin_thread([&executor]() { executor.spin(); });

  // Keep main thread alive
  while (rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Cleanup
  executor.cancel();
  spin_thread.join();

  rclcpp::shutdown();
  return 0;
}
