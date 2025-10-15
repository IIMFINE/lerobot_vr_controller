#ifndef CHASSIS_CONTROLLER_H
#define CHASSIS_CONTROLLER_H

#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <string>

namespace lerobot {

class ChassisController {
public:
  explicit ChassisController(rclcpp::Node::SharedPtr node,
                             const std::string &config_path);
  ~ChassisController();

  void ProcessJoyMsg(const sensor_msgs::msg::Joy::SharedPtr msg);

  geometry_msgs::msg::Twist GetTwistMsg() const;

private:
  void LoadConfig(const std::string &config_path);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
  geometry_msgs::msg::Twist twist_msg_;

  double linear_vel_rate_;
  double angular_vel_rate_;
  std::string vel_cmd_topic_;
};

} // namespace lerobot

#endif // CHASSIS_CONTROLLER_H
