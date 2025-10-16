#include "chassis_controller.h"
#include "log.h"
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
    LE_LOG_INFO << "Config path is empty, using default values" << std::endl;
    return;
  }

  std::ifstream file(config_path);
  if (!file.good()) {
    LE_LOG_ERROR << "Failed to open config file: " << config_path << std::endl;
    return;
  }

  try {
    YAML::Node config = YAML::LoadFile(config_path);
    
    if (config["linear_vel_rate"]) {
      linear_vel_rate_ = config["linear_vel_rate"].as<double>();
      LE_LOG_INFO << "Loaded linear_vel_rate: " << linear_vel_rate_ << std::endl;
    }
    
    if (config["angular_vel_rate"]) {
      angular_vel_rate_ = config["angular_vel_rate"].as<double>();
      LE_LOG_INFO << "Loaded angular_vel_rate: " << angular_vel_rate_ << std::endl;
    }
    
    if (config["vel_cmd_topic"]) {
      vel_cmd_topic_ = config["vel_cmd_topic"].as<std::string>();
      LE_LOG_INFO << "Loaded vel_cmd_topic: " << vel_cmd_topic_ << std::endl;
    }
  } catch (const YAML::Exception& e) {
    LE_LOG_ERROR << "Failed to parse config file: " << e.what() << std::endl;
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

  twist_msg_.angular.z = msg->axes[2] * angular_vel_rate_;
  twist_msg_.linear.x = msg->axes[3] * linear_vel_rate_;

  LE_LOG_INFO << "Publishing Twist: linear.x=" << twist_msg_.linear.x
              << ", angular.z=" << twist_msg_.angular.z << std::endl;
  twist_publisher_->publish(twist_msg_);
}

geometry_msgs::msg::Twist ChassisController::GetTwistMsg() const {
  return twist_msg_;
}

} // namespace lerobot
