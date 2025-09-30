#ifndef ROBOT_CONTROL_INTERFACE_H_
#define ROBOT_CONTROL_INTERFACE_H_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <shared_mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "interface_type.h"
#include "log.h"
#include "robot_communicate_interface.h"

namespace lerobot_vr_controller {

/**
 * @brief Interface for managing robot joint and gripper command queues.
 *
 * This class provides thread-safe mechanisms for enqueuing and processing joint
 * and gripper commands using separate queues for each robot component. Commands
 * are processed asynchronously by dedicated worker threads.
 */
class RobotControlInterface {
public:
  /**
   * @brief Constructor for RobotControlInterface.
   * @param node Shared pointer to ROS2 node
   * @param joint_motor_config_file_path Path to joint motor configuration file
   * @param motor_calibration_file_path Path to motor calibration file
   * (optional)
   * @param motor_cmd_topic Topic name for publishing motor positions
   * @param motor_state_topic Topic name for subscribing to motor state
   */
  explicit RobotControlInterface(
      std::shared_ptr<rclcpp::Node> node,
      const std::string &joint_motor_config_file_path,
      const std::string &motor_calibration_file_path,
      const std::string &motor_cmd_topic = "/robot_control/motor_cmd",
      const std::string &motor_state_topic = "/robot_control/motor_state");

  /**
   * @brief Destructor that properly shuts down all worker threads.
   */
  ~RobotControlInterface();

  // Delete copy constructor and assignment operator (non-copyable)
  RobotControlInterface(const RobotControlInterface &) = delete;
  RobotControlInterface &operator=(const RobotControlInterface &) = delete;

  // Allow move semantics
  RobotControlInterface(RobotControlInterface &&) = default;
  RobotControlInterface &operator=(RobotControlInterface &&) = default;

  /**
   * @brief Initialize the robot control interface.
   * @return true if initialization successful, false otherwise.
   */
  bool Initialize();

  /**
   * @brief Start the control interface and worker threads.
   */
  void Start();

  /**
   * @brief Stop the control interface and shutdown worker threads.
   */
  void Stop();

  /**
   * @brief Enqueue a joint command for execution.
   * @param joint_cmd Joint state command to enqueue.
   * @return true if command was enqueued successfully.
   */
  bool EnqueueJointCommand(const CusJointCmd &joint_cmd);

  /**
   * @brief Enqueue a gripper command for execution.
   * @param gripper_cmd Gripper state command to enqueue.
   * @return true if command was enqueued successfully.
   */
  bool EnqueueGripperCommand(const CusJointCmd &gripper_cmd);

  /**
   * @brief Dequeue the next joint command.
   * @param joint_cmd Output parameter to store the dequeued command.
   * @return true if a command was dequeued, false if queue is empty.
   */
  bool DequeueJointCommand(CusJointCmd &joint_cmd);

  /**
   * @brief Dequeue the next gripper command.
   * @param gripper_cmd Output parameter to store the dequeued command.
   * @return true if a command was dequeued, false if queue is empty.
   */
  bool DequeueGripperCommand(CusJointCmd &gripper_cmd);

  /**
   * @brief Get the number of pending joint commands.
   * @return Number of pending commands in the queue.
   */
  size_t GetJointCommandQueueSize() const;

  /**
   * @brief Get the number of pending gripper commands.
   * @return Number of pending commands in the queue.
   */
  size_t GetGripperCommandQueueSize() const;

  /**
   * @brief Clear all pending commands.
   */
  void ClearAllCommands();

  /**
   * @brief Check if the control interface is currently running.
   * @return true if running, false otherwise.
   */
  bool IsRunning() const { return is_running_.load(); }

  /**
   * @brief Get current joint position state from robot.
   *
   * This method retrieves the current motor positions from the robot
   * communication interface and converts them to joint positions using the
   * joint-motor converter.
   *
   * @return JointPositionState containing current joint positions and
   * timestamp. If no motor state is available, returns empty joint positions.
   */
  JointPositionState GetJointPositionState() const;

  /**
   * @brief Attempt to fuse joint and gripper commands if their timestamps are
   * close.
   *
   * This method checks the front elements of both joint and gripper command
   * queues. If both queues have commands and their timestamps
   * differ by less than 1ms, they are combined into a single CusJointCmd
   * structure. If one queue is empty, it attempts to fuse with the last
   * recorded command.
   *
   * @return std::optional<CusJointCmd> containing the fused command if fusion
   *         occurs, std::nullopt otherwise.
   */
  std::optional<CusJointCmd> FuseCommand();

  /**
   * @brief Control robot with 100Hz frequency using timer.
   * This method runs in a separate thread and continuously processes
   * fused commands at 100Hz rate.
   */
  void ControlRobot();

  /**
   * @brief Start the control robot timer thread.
   */
  void StartControlThread();

  /**
   * @brief Stop the control robot timer thread.
   */
  void StopControlThread();

private:
  // RobotCommunicateInterface for robot communication
  std::unique_ptr<RobotCommunicateInterface> robot_communicate_interface_;

  // Mutexes and condition variables for thread synchronization
  mutable std::shared_mutex joint_cmd_queue_mutex_;
  std::condition_variable_any joint_cmd_queue_cond_;
  std::deque<CusJointCmd> joint_cmd_queue_;

  mutable std::shared_mutex gripper_cmd_queue_mutex_;
  std::condition_variable_any gripper_cmd_queue_cond_;
  std::deque<CusJointCmd> gripper_cmd_queue_;

  // Control flags
  std::atomic<bool> is_running_{false};
  std::atomic<bool> should_stop_{false};

  // Control robot timer thread
  std::thread control_thread_;
  std::atomic<bool> control_thread_running_{false};

  // Last recorded commands for fusion when queues are empty
  mutable std::shared_mutex last_cmd_mutex_;
  std::deque<CusJointCmd> last_joint_cmd_;
  std::deque<CusJointCmd> last_gripper_cmd_;

  // Maximum queue sizes to prevent memory issues
  static constexpr size_t kMaxQueueSize = 100;

  // Time difference threshold for command fusion (1ms in nanoseconds)
  static constexpr uint64_t kFusionThresholdNs = 5'000'000;

  // Control frequency: 100Hz = 10ms period
  static constexpr std::chrono::milliseconds kControlPeriod{10};
};

} // namespace lerobot_vr_controller

#endif // ROBOT_CONTROL_INTERFACE_H_
