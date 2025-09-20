#ifndef ROBOT_CONTROL_INTERFACE_H_
#define ROBOT_CONTROL_INTERFACE_H_

#include <atomic>
#include <condition_variable>
#include <deque>
#include <map>
#include <memory>
#include <shared_mutex>
#include <string>
#include <thread>
#include <vector>

#include "log.h"
#include "sensor_msgs/msg/joint_state.hpp"

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
   * @brief Default constructor for RobotControlInterface.
   */
  RobotControlInterface() = default;

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
   * @param robot_name Name of the robot (e.g., "left_arm", "right_arm").
   * @param joint_cmd Joint state command to enqueue.
   * @return true if command was enqueued successfully.
   */
  bool EnqueueJointCommand(const std::string &robot_name,
                           const sensor_msgs::msg::JointState &joint_cmd);

  /**
   * @brief Enqueue a gripper command for execution.
   * @param robot_name Name of the robot gripper.
   * @param gripper_cmd Gripper state command to enqueue.
   * @return true if command was enqueued successfully.
   */
  bool EnqueueGripperCommand(const std::string &robot_name,
                             const sensor_msgs::msg::JointState &gripper_cmd);

  /**
   * @brief Dequeue the next joint command for a specific robot.
   * @param robot_name Name of the robot.
   * @param joint_cmd Output parameter to store the dequeued command.
   * @return true if a command was dequeued, false if queue is empty.
   */
  bool DequeueJointCommand(const std::string &robot_name,
                           sensor_msgs::msg::JointState &joint_cmd);

  /**
   * @brief Dequeue the next gripper command for a specific robot.
   * @param robot_name Name of the robot.
   * @param gripper_cmd Output parameter to store the dequeued command.
   * @return true if a command was dequeued, false if queue is empty.
   */
  bool DequeueGripperCommand(const std::string &robot_name,
                             sensor_msgs::msg::JointState &gripper_cmd);

  /**
   * @brief Get the number of pending joint commands for a specific robot.
   * @param robot_name Name of the robot.
   * @return Number of pending commands in the queue.
   */
  size_t GetJointCommandQueueSize(const std::string &robot_name) const;

  /**
   * @brief Get the number of pending gripper commands for a specific robot.
   * @param robot_name Name of the robot.
   * @return Number of pending commands in the queue.
   */
  size_t GetGripperCommandQueueSize(const std::string &robot_name) const;

  /**
   * @brief Clear all pending commands for a specific robot.
   * @param robot_name Name of the robot.
   */
  void ClearAllCommands(const std::string &robot_name);

  /**
   * @brief Check if the control interface is currently running.
   * @return true if running, false otherwise.
   */
  bool IsRunning() const { return is_running_.load(); }

private:
  // Thread-safe command queues
  std::map<std::string, std::deque<sensor_msgs::msg::JointState>>
      joint_cmd_queue_;
  std::map<std::string, std::deque<sensor_msgs::msg::JointState>>
      gripper_cmd_queue_;

  // Mutexes and condition variables for thread synchronization
  mutable std::shared_mutex joint_cmd_queue_mutex_;
  std::condition_variable_any joint_cmd_queue_cond_;

  mutable std::shared_mutex gripper_cmd_queue_mutex_;
  std::condition_variable_any gripper_cmd_queue_cond_;

  // Control flags
  std::atomic<bool> is_running_{false};
  std::atomic<bool> should_stop_{false};

  // Maximum queue sizes to prevent memory issues
  static constexpr size_t kMaxQueueSize = 1000;
};

} // namespace lerobot_vr_controller

#endif // ROBOT_CONTROL_INTERFACE_H_
