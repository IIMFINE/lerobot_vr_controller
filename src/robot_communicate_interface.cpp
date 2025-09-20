#include "robot_communicate_interface.h"

#include <iomanip>
#include <nlohmann/json.hpp>
#include <sstream>

namespace lerobot_vr_controller {

RobotCommunicateInterface::RobotCommunicateInterface(
    std::shared_ptr<rclcpp::Node> node,
    std::shared_ptr<JointMotorConvert> joint_motor_converter,
    const std::string &motor_cmd_topic, const std::string &motor_state_topic)
    : node_(std::move(node)),
      joint_motor_converter_(std::move(joint_motor_converter)),
      motor_cmd_topic_(motor_cmd_topic), motor_state_topic_(motor_state_topic) {

  if (!node_ || !joint_motor_converter_) {
    LE_LOG_ERROR << "Invalid node or joint_motor_converter provided to "
                    "RobotCommunicateInterface"
                 << std::endl;
    return;
  }

  // Create publisher
  publisher_ =
      node_->create_publisher<std_msgs::msg::String>(motor_cmd_topic_, 10);

  // Create subscriber
  subscriber_ = node_->create_subscription<std_msgs::msg::String>(
      motor_state_topic_, 10,
      std::bind(&RobotCommunicateInterface::MotorStateCallback, this,
                std::placeholders::_1));

  LE_LOG_INFO << "RobotCommunicateInterface initialized with cmd topic: "
              << motor_cmd_topic_ << " and state topic: " << motor_state_topic_
              << std::endl;
}

bool RobotCommunicateInterface::UpdateJointCmd(const CusJointCmd &joint_cmd) {
  if (joint_cmd.joints.empty()) {
    LE_LOG_ERROR << "Empty joint positions provided" << std::endl;
    return false;
  }

  // Convert CusJointCmd to unordered_map for processing
  std::unordered_map<std::string, double> joint_positions;
  for (const auto &[joint_name, position] : joint_cmd.joints) {
    joint_positions[joint_name] = position;
  }

  // Validate joint names
  for (const auto &[joint_name, position] : joint_positions) {
    if (!joint_motor_converter_->IsValidJoint(joint_name)) {
      LE_LOG_ERROR << "Invalid joint name: " << joint_name << std::endl;
      return false;
    }
  }

  // Convert to motor positions
  last_motor_cmd_ = ConvertJointsToMotors(joint_positions);

  LE_LOG_INFO_T(1s) << "Updated joint positions for " << joint_cmd.joints.size()
                    << " joints with timestamp: " << joint_cmd.timestamp_ns
                    << std::endl;

  return true;
}

bool RobotCommunicateInterface::PublishMotorCmd() {
  if (!publisher_) {
    LE_LOG_ERROR << "Publisher not initialized" << std::endl;
    return false;
  }

  if (last_motor_cmd_.empty()) {
    LE_LOG_ERROR << "No motor positions to publish" << std::endl;
    return false;
  }

  // Create message
  auto message = std_msgs::msg::String();
  message.data = FormatMotorPositionsAsJson(last_motor_cmd_);

  // Publish message
  publisher_->publish(message);

  LE_LOG_INFO_T(5s) << "Published motor positions: " << message.data
                    << std::endl;

  return true;
}

std::unordered_map<std::string, int>
RobotCommunicateInterface::GetCurrentMotorPositions() const {
  return last_motor_cmd_;
}

std::string RobotCommunicateInterface::GetLastMotorState() const {
  return last_motor_state_;
}

void RobotCommunicateInterface::MotorStateCallback(
    const std_msgs::msg::String::SharedPtr msg) {
  last_motor_state_ = msg->data;
  last_received_motor_positions_ =
      ParseMotorPositionsFromJson(last_motor_state_);
  LE_LOG_INFO_T(5s) << "Received motor state: " << last_motor_state_
                    << ", parsed " << last_received_motor_positions_.size()
                    << " motor positions" << std::endl;
}

void RobotCommunicateInterface::Start() {
  LE_LOG_INFO << "Starting RobotCommunicateInterface" << std::endl;
}

void RobotCommunicateInterface::Stop() {
  LE_LOG_INFO << "Stopping RobotCommunicateInterface" << std::endl;
}

std::unordered_map<std::string, int>
RobotCommunicateInterface::ConvertJointsToMotors(
    const std::unordered_map<std::string, double> &joint_positions) const {

  std::unordered_map<std::string, int> motor_positions;

  for (const auto &[joint_name, joint_position] : joint_positions) {
    try {
      const double motor_position_double =
          joint_motor_converter_->JointToMotorPosition(joint_name,
                                                       joint_position);

      // Round to nearest integer for motor position
      motor_positions[joint_name] =
          static_cast<int>(std::round(motor_position_double));

    } catch (const std::exception &e) {
      LE_LOG_ERROR << "Failed to convert joint '" << joint_name
                   << "' to motor position: " << e.what() << std::endl;
    }
  }

  return motor_positions;
}

std::string RobotCommunicateInterface::FormatMotorPositionsAsJson(
    const std::unordered_map<std::string, int> &motor_positions) const {

  if (motor_positions.empty()) {
    return "{}";
  }

  std::ostringstream json_stream;
  json_stream << "{";

  bool first = true;
  for (const auto &[joint_name, motor_position] : motor_positions) {
    if (!first) {
      json_stream << ", ";
    }
    json_stream << "\"" << joint_name << ".pos\": " << motor_position;
    first = false;
  }

  json_stream << "}";
  return json_stream.str();
}

std::unordered_map<std::string, int>
RobotCommunicateInterface::ParseMotorPositionsFromJson(
    const std::string &json_str) const {
  std::unordered_map<std::string, int> motor_positions;

  if (json_str.empty()) {
    LE_LOG_ERROR << "Empty JSON string provided for parsing" << std::endl;
    return motor_positions;
  }

  try {
    // Parse JSON using nlohmann::json
    nlohmann::json j = nlohmann::json::parse(json_str);

    // Iterate through all key-value pairs
    for (auto &[key, value] : j.items()) {
      // Remove .pos suffix from key to get joint name
      std::string joint_name = key;
      const std::string suffix = ".pos";
      if (joint_name.length() >= suffix.length() &&
          joint_name.substr(joint_name.length() - suffix.length()) == suffix) {
        joint_name =
            joint_name.substr(0, joint_name.length() - suffix.length());
      }

      // Convert value to integer
      if (value.is_number_integer()) {
        motor_positions[joint_name] = value.get<int>();
        LE_LOG_INFO_T(5s) << "Parsed motor position: " << joint_name << " = "
                          << motor_positions[joint_name] << std::endl;
      } else if (value.is_number_float()) {
        motor_positions[joint_name] =
            static_cast<int>(std::round(value.get<double>()));
        LE_LOG_INFO_T(5s) << "Parsed motor position (rounded): " << joint_name
                          << " = " << motor_positions[joint_name] << std::endl;
      } else {
        LE_LOG_ERROR << "Invalid value type for key '" << key
                     << "': expected number, got " << value.type_name()
                     << std::endl;
      }
    }

  } catch (const nlohmann::json::parse_error &e) {
    LE_LOG_ERROR << "JSON parse error: " << e.what() << std::endl;
  } catch (const nlohmann::json::type_error &e) {
    LE_LOG_ERROR << "JSON type error: " << e.what() << std::endl;
  } catch (const std::exception &e) {
    LE_LOG_ERROR << "Failed to parse JSON string '" << json_str
                 << "': " << e.what() << std::endl;
  }

  return motor_positions;
}

std::unordered_map<std::string, int>
RobotCommunicateInterface::GetLastReceivedMotorPositions() const {
  return last_received_motor_positions_;
}

std::vector<std::pair<std::string, double>>
RobotCommunicateInterface::ConvertMotorsToJoints(
    const std::unordered_map<std::string, int> &motor_positions) const {
  std::vector<std::pair<std::string, double>> joint_positions;
  joint_positions.reserve(motor_positions.size());

  if (!joint_motor_converter_) {
    LE_LOG_ERROR << "JointMotorConverter is not initialized" << std::endl;
    return joint_positions;
  }

  // Convert each motor position to joint position
  for (const auto &motor_pair : motor_positions) {
    const std::string &joint_name = motor_pair.first;
    int motor_position = motor_pair.second;

    try {
      // Convert motor position to joint position using the converter
      double joint_position = joint_motor_converter_->MotorToJointPosition(
          joint_name, static_cast<double>(motor_position));

      joint_positions.emplace_back(joint_name, joint_position);

      LE_LOG_INFO_T(5s) << "Converted motor position " << motor_position
                        << " to joint position " << joint_position
                        << " for joint: " << joint_name << std::endl;
    } catch (const std::exception &e) {
      LE_LOG_ERROR << "Failed to convert motor position for joint "
                   << joint_name << ": " << e.what() << std::endl;
    }
  }

  return joint_positions;
}

JointPositionState RobotCommunicateInterface::GetJointPositionState() const {
  JointPositionState joint_state;

  // Get current motor positions from robot
  auto motor_positions = GetLastReceivedMotorPositions();

  if (motor_positions.empty()) {
    LE_LOG_ERROR_T(5s) << "No motor positions available from robot"
                       << std::endl;
    return joint_state;
  }

  // Convert motor positions to joint positions
  joint_state.joint_positions = ConvertMotorsToJoints(motor_positions);

  LE_LOG_INFO_T(5s) << "Retrieved joint positions for "
                    << joint_state.joint_positions.size() << " joints"
                    << std::endl;

  return joint_state;
}

} // namespace lerobot_vr_controller
