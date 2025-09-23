#ifndef ROBOT_COMMUNICATE_INTERFACE_H_
#define ROBOT_COMMUNICATE_INTERFACE_H_

#include <memory>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "interface_type.h"
#include "joint_motor_convert.h"
#include "log.h"

namespace lerobot_vr_controller {

/**
 * @brief Interface for robot communication that converts joint positions to
 * motor positions and publishes them as JSON formatted strings via ROS2.
 */
class RobotCommunicateInterface {
public:
  /**
   * @brief Constructor that initializes the communication interface.
   * @param node Shared pointer to ROS2 node for publishing
   * @param joint_motor_converter Shared pointer to joint-motor converter
   * @param motor_cmd_topic Topic name for publishing motor positions (default:
   * "/robot_control/motor_cmd")
   * @param motor_state_topic Topic name for subscribing to motor state
   * (default:
   * "/robot_control/motor_state")
   */
  explicit RobotCommunicateInterface(
      std::shared_ptr<rclcpp::Node> node,
      std::shared_ptr<JointMotorConvert> joint_motor_converter,
      const std::string &motor_cmd_topic = "/robot_control/motor_cmd",
      const std::string &motor_state_topic = "/robot_control/motor_state");

  /**
   * @brief Destructor
   */
  ~RobotCommunicateInterface() = default;

  /**
   * @brief Update joint positions and convert them to motor positions
   * @param joint_cmd Custom joint command containing joint positions and
   * timestamp
   * @return True if update was successful, false otherwise
   */
  bool UpdateJointCmd(const CusJointCmd &joint_cmd);

  /**
   * @brief Publish current motor positions as JSON string
   * @return True if publish was successful, false otherwise
   */
  bool PublishMotorCmd();

  /**
   * @brief Get current motor positions
   * @return Map of joint names to motor positions
   */
  std::unordered_map<std::string, int> GetCurrentMotorPositions() const;

  /**
   * @brief Get last received motor state
   * @return Last received motor state as string
   */
  std::string GetLastMotorState() const;

  /**
   * @brief Get last received motor positions parsed from JSON
   * @return Map of joint names to motor positions
   */
  std::unordered_map<std::string, int> GetLastReceivedMotorPositions() const;

  /**
   * @brief Get current joint position state from robot.
   *
   * This method retrieves the current motor positions and converts them
   * to joint positions using the joint-motor converter.
   *
   * @return JointPositionState containing current joint positions and
   * timestamp. If no motor state is available, returns empty joint positions.
   */
  JointPositionState GetJointPositionState() const;

  /**
   * @brief Convert motor positions to joint positions
   * @param motor_positions Map of joint names to motor positions
   * @return Vector of joint name-position pairs in radians
   */
  std::vector<std::pair<std::string, double>> ConvertMotorsToJoints(
      const std::unordered_map<std::string, int> &motor_positions) const;

  /**
   * @brief Start the robot communication interface
   */
  void Start();

  /**
   * @brief Stop the robot communication interface
   */
  void Stop();

private:
  /**
   * @brief Convert joint positions to motor positions using the converter
   * @param joint_positions Joint positions in radians
   * @return Motor positions as integers
   */
  std::unordered_map<std::string, int> ConvertJointsToMotors(
      const std::unordered_map<std::string, double> &joint_positions) const;

  /**
   * @brief Format motor positions as JSON string
   * @param motor_positions Motor positions to format
   * @return JSON formatted string
   */
  std::string FormatMotorPositionsAsJson(
      const std::unordered_map<std::string, int> &motor_positions) const;

  /**
   * @brief Callback function for motor state subscription
   * @param msg Received motor state message
   *            Message type: std_msgs::msg::String
   *            Data format example:
   *            data: '{"shoulder_pan.pos": 1834, "shoulder_lift.pos": 2142,
   * "elbow_flex.pos": 1818, "wrist_flex.pos": 2319, "wrist_roll.pos": 2329,
   * "gripper.pos": 2047}'
   */
  void MotorStateCallback(const std_msgs::msg::String::SharedPtr msg);

  /**
   * @brief Parse JSON string to extract motor positions
   * @param json_str JSON formatted string containing motor positions
   * @return Map of joint names to motor positions
   */
  std::unordered_map<std::string, int> ParseMotorPositionsFromJson(
      const std::string &json_str) const;

  // ROS2 components
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;

  // Joint-motor conversion
  std::shared_ptr<JointMotorConvert> joint_motor_converter_;

  // Current state
  std::unordered_map<std::string, int> last_motor_cmd_;
  std::string last_motor_state_;
  std::unordered_map<std::string, int> last_received_motor_positions_;
  mutable std::shared_mutex last_received_motor_positions_mutex_;

  // Configuration
  std::string motor_cmd_topic_;
  std::string motor_state_topic_;
};

} // namespace lerobot_vr_controller

#endif // ROBOT_COMMUNICATE_INTERFACE_H_
