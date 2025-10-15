#include "chassis_controller.h"
#include <yaml-cpp/yaml.h>
#include <fstream>

namespace lerobot {

ChassisController::ChassisController(rclcpp::Node::SharedPtr node, const std::string& config_path)
    : node_(node), linear_vel_rate_(1.0), angular_vel_rate_(1.0), vel_cmd_topic_("cmd_vel") {
  twist_msg_.linear.x = 0.0;
  twist_msg_.linear.y = 0.0;
  twist_msg_.linear.z = 0.0;
  twist_msg_.angular.x = 0.0;
  twist_msg_.angular.y = 0.0;
  twist_msg_.angular.z = 0.0;

  LoadConfig(config_path);

  twist_publisher_ =
      node_->create_publisher<geometry_msgs::msg::Twist>(vel_cmd_topic_, 10);
}

ChassisController::~ChassisController() {}

void ChassisController::LoadConfig(const std::string& config_path) {
  if (config_path.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Config path is empty, using default values");
    return;
  }

  std::ifstream file(config_path);
  if (!file.good()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to open config file: %s", config_path.c_str());
    return;
  }

  try {
    YAML::Node config = YAML::LoadFile(config_path);
    
    if (config["linear_vel_rate"]) {
      linear_vel_rate_ = config["linear_vel_rate"].as<double>();
      RCLCPP_INFO(node_->get_logger(), "Loaded linear_vel_rate: %f", linear_vel_rate_);
    }
    
    if (config["angular_vel_rate"]) {
      angular_vel_rate_ = config["angular_vel_rate"].as<double>();
      RCLCPP_INFO(node_->get_logger(), "Loaded angular_vel_rate: %f", angular_vel_rate_);
    }
    
    if (config["vel_cmd_topic"]) {
      vel_cmd_topic_ = config["vel_cmd_topic"].as<std::string>();
      RCLCPP_INFO(node_->get_logger(), "Loaded vel_cmd_topic: %s", vel_cmd_topic_.c_str());
    }
  } catch (const YAML::Exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to parse config file: %s", e.what());
  }
}

void ChassisController::ProcessJoyMsg(
    const sensor_msgs::msg::Joy::SharedPtr msg) {
  if (!msg) {
    return;
  }

  if (msg->axes.size() < 4) {
    return;
  }

  twist_msg_.linear.x = msg->axes[2] * linear_vel_rate_;
  twist_msg_.angular.z = msg->axes[3] * angular_vel_rate_;

  twist_publisher_->publish(twist_msg_);
}

geometry_msgs::msg::Twist ChassisController::GetTwistMsg() const {
  return twist_msg_;
}

} // namespace lerobot
