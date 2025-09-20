#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace lerobot_vr_controller {
/**
 * @brief Custom joint command structure.
 *
 * Contains a timestamp in nanoseconds (steady clock) and joint name-position
 * pairs.
 */
struct CusJointCmd {
  uint64_t timestamp_ns; ///< Timestamp in nanoseconds using steady clock
  std::vector<std::pair<std::string, double>>
      joints; ///< Joint name-position pairs

  /**
   * @brief Default constructor with current timestamp.
   */
  CusJointCmd()
      : timestamp_ns(
            std::chrono::steady_clock::now().time_since_epoch().count()) {}

  /**
   * @brief Constructor with specified timestamp.
   * @param ts Timestamp in nanoseconds.
   */
  explicit CusJointCmd(uint64_t ts) : timestamp_ns(ts) {}

  /**
   * @brief Constructor with joints and current timestamp.
   * @param joint_data Joint name-position pairs.
   */
  explicit CusJointCmd(
      const std::vector<std::pair<std::string, double>> &joint_data)
      : timestamp_ns(
            std::chrono::steady_clock::now().time_since_epoch().count()),
        joints(joint_data) {}

  /**
   * @brief Constructor with timestamp and joints.
   * @param ts Timestamp in nanoseconds.
   * @param joint_data Joint name-position pairs.
   */
  CusJointCmd(uint64_t ts,
              const std::vector<std::pair<std::string, double>> &joint_data)
      : timestamp_ns(ts), joints(joint_data) {}
};

/**
 * @brief Joint position state structure.
 *
 * Contains current joint positions with timestamp.
 */
struct JointPositionState {
  uint64_t timestamp_ns; ///< Timestamp in nanoseconds using steady clock
  std::vector<std::pair<std::string, double>>
      joint_positions; ///< Joint name-position pairs in radians

  /**
   * @brief Default constructor with current timestamp.
   */
  JointPositionState()
      : timestamp_ns(
            std::chrono::steady_clock::now().time_since_epoch().count()) {}

  /**
   * @brief Constructor with joint positions and current timestamp.
   * @param positions Joint name-position pairs.
   */
  explicit JointPositionState(
      const std::vector<std::pair<std::string, double>> &positions)
      : timestamp_ns(
            std::chrono::steady_clock::now().time_since_epoch().count()),
        joint_positions(positions) {}

  /**
   * @brief Constructor with timestamp and joint positions.
   * @param ts Timestamp in nanoseconds.
   * @param positions Joint name-position pairs.
   */
  JointPositionState(
      uint64_t ts, const std::vector<std::pair<std::string, double>> &positions)
      : timestamp_ns(ts), joint_positions(positions) {}
};

} // namespace lerobot_vr_controller
