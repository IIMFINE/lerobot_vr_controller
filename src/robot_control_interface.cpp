#include "robot_control_interface.h"

#include <mutex>

#include "joint_motor_convert.h"

namespace lerobot_vr_controller {

RobotControlInterface::RobotControlInterface(
    std::shared_ptr<rclcpp::Node> node,
    const std::string &joint_motor_config_file_path,
    const std::string &motor_calibration_file_path,
    const std::string &motor_cmd_topic) {

  // Create RobotCommunicateInterface instance
  robot_communicate_interface_ = std::make_unique<RobotCommunicateInterface>(
      node,
      std::make_shared<JointMotorConvert>(joint_motor_config_file_path,
                                          motor_calibration_file_path),
      motor_cmd_topic);

  LE_LOG_INFO << "RobotControlInterface constructed with config: "
              << joint_motor_config_file_path << std::endl;
}

RobotControlInterface::~RobotControlInterface() {
  Stop();
  LE_LOG_INFO << "RobotControlInterface destroyed" << std::endl;
}

bool RobotControlInterface::Initialize() { return true; }

void RobotControlInterface::Start() {
  if (is_running_.load()) {
    LE_LOG_INFO << "RobotControlInterface is already running" << std::endl;
    return;
  }

  LE_LOG_INFO << "Starting RobotControlInterface..." << std::endl;

  should_stop_.store(false);
  is_running_.store(true);

  // Start auto publishing if robot communicate interface exists
  if (robot_communicate_interface_) {
    robot_communicate_interface_->Start();
  }

  // Start control robot timer thread
  StartControlThread();

  LE_LOG_INFO << "RobotControlInterface started successfully" << std::endl;
}

void RobotControlInterface::Stop() {
  if (!is_running_.load()) {
    return;
  }

  LE_LOG_INFO << "Stopping RobotControlInterface..." << std::endl;

  should_stop_.store(true);
  is_running_.store(false);

  // Stop control robot timer thread
  StopControlThread();

  // Stop auto publishing and robot communicate interface
  if (robot_communicate_interface_) {
    robot_communicate_interface_->Stop();
  }

  // Clear all command queues
  ClearAllCommands();

  // Notify all waiting threads
  joint_cmd_queue_cond_.notify_all();
  gripper_cmd_queue_cond_.notify_all();

  LE_LOG_INFO << "RobotControlInterface stopped and all queues cleared"
              << std::endl;
}

bool RobotControlInterface::EnqueueJointCommand(const CusJointCmd &joint_cmd) {
  // Return false if the interface is not running or should stop
  if (!is_running_.load() || should_stop_.load()) {
    LE_LOG_INFO_T(5s)
        << "EnqueueJointCommand: Interface is stopped, rejecting command"
        << std::endl;
    return false;
  }

  std::unique_lock<std::shared_mutex> lock(joint_cmd_queue_mutex_);

  if (joint_cmd_queue_.size() >= kMaxQueueSize) {
    // Pop the front element when queue is full
    joint_cmd_queue_.pop_front();
    LE_LOG_ERROR_T(1s)
        << "Joint command queue is full, popped front element (size: "
        << joint_cmd_queue_.size() << ")" << std::endl;
  }

  joint_cmd_queue_.push_back(joint_cmd);
  lock.unlock();

  joint_cmd_queue_cond_.notify_one();
  LE_LOG_INFO_T(1s) << "Enqueued joint command (queue size: "
                    << joint_cmd_queue_.size() << ")" << std::endl;
  return true;
}

bool RobotControlInterface::EnqueueGripperCommand(
    const CusJointCmd &gripper_cmd) {
  // Return false if the interface is not running or should stop
  if (!is_running_.load() || should_stop_.load()) {
    LE_LOG_INFO_T(5s)
        << "EnqueueGripperCommand: Interface is stopped, rejecting command"
        << std::endl;
    return false;
  }

  std::unique_lock<std::shared_mutex> lock(gripper_cmd_queue_mutex_);

  if (gripper_cmd_queue_.size() >= kMaxQueueSize) {
    // Pop the front element when queue is full
    gripper_cmd_queue_.pop_front();
    LE_LOG_ERROR_T(1s)
        << "Gripper command queue is full, popped front element (size: "
        << gripper_cmd_queue_.size() << ")" << std::endl;
  }

  gripper_cmd_queue_.push_back(gripper_cmd);
  lock.unlock();

  gripper_cmd_queue_cond_.notify_one();
  LE_LOG_INFO_T(1s) << "Enqueued gripper command (queue size: "
                    << gripper_cmd_queue_.size() << ")" << std::endl;
  return true;
}

bool RobotControlInterface::DequeueJointCommand(CusJointCmd &joint_cmd) {
  std::unique_lock<std::shared_mutex> lock(joint_cmd_queue_mutex_);

  if (joint_cmd_queue_.empty()) {
    return false;
  }

  joint_cmd = std::move(joint_cmd_queue_.front());
  joint_cmd_queue_.pop_front();

  LE_LOG_INFO_T(1s) << "Dequeued joint command (remaining: "
                    << joint_cmd_queue_.size() << ")" << std::endl;
  return true;
}

bool RobotControlInterface::DequeueGripperCommand(CusJointCmd &gripper_cmd) {
  std::unique_lock<std::shared_mutex> lock(gripper_cmd_queue_mutex_);

  if (gripper_cmd_queue_.empty()) {
    return false;
  }

  gripper_cmd = std::move(gripper_cmd_queue_.front());
  gripper_cmd_queue_.pop_front();

  LE_LOG_INFO_T(1s) << "Dequeued gripper command (remaining: "
                    << gripper_cmd_queue_.size() << ")" << std::endl;
  return true;
}

size_t RobotControlInterface::GetJointCommandQueueSize() const {
  std::shared_lock<std::shared_mutex> lock(joint_cmd_queue_mutex_);
  return joint_cmd_queue_.size();
}

size_t RobotControlInterface::GetGripperCommandQueueSize() const {
  std::shared_lock<std::shared_mutex> lock(gripper_cmd_queue_mutex_);
  return gripper_cmd_queue_.size();
}

void RobotControlInterface::ClearAllCommands() {
  {
    std::unique_lock<std::shared_mutex> joint_lock(joint_cmd_queue_mutex_);
    joint_cmd_queue_.clear();
  }

  {
    std::unique_lock<std::shared_mutex> gripper_lock(gripper_cmd_queue_mutex_);
    gripper_cmd_queue_.clear();
  }

  LE_LOG_INFO << "Cleared all commands" << std::endl;
}

std::optional<CusJointCmd> RobotControlInterface::FuseCommand() {
  // Use scoped locking to acquire all mutexes in a consistent order
  // to avoid potential deadlocks
  std::scoped_lock<std::shared_mutex, std::shared_mutex, std::shared_mutex>
      lock(joint_cmd_queue_mutex_, gripper_cmd_queue_mutex_, last_cmd_mutex_);

  // Helper lambda to check if two commands can be fused
  auto can_fuse = [this](const CusJointCmd &cmd1, const CusJointCmd &cmd2) {
    const uint64_t time_diff = (cmd1.timestamp_ns > cmd2.timestamp_ns)
                                   ? (cmd1.timestamp_ns - cmd2.timestamp_ns)
                                   : (cmd2.timestamp_ns - cmd1.timestamp_ns);

    // TODO: delete it Print time_diff in microseconds at 500ms frequency
    LE_LOG_INFO_T(500ms) << "FuseCommand: time_diff = " << (time_diff / 1000.0)
                         << " us" << std::endl;

    return time_diff <= kFusionThresholdNs;
  };

  // Helper lambda to create fused command
  auto create_fused_cmd = [](const CusJointCmd &cmd1, const CusJointCmd &cmd2) {
    CusJointCmd fused_cmd;
    fused_cmd.timestamp_ns = std::min(cmd1.timestamp_ns, cmd2.timestamp_ns);
    fused_cmd.joints.reserve(cmd1.joints.size() + cmd2.joints.size());
    fused_cmd.joints.insert(fused_cmd.joints.end(), cmd1.joints.begin(),
                            cmd1.joints.end());
    fused_cmd.joints.insert(fused_cmd.joints.end(), cmd2.joints.begin(),
                            cmd2.joints.end());
    return fused_cmd;
  };

  const bool has_joint_cmd = !joint_cmd_queue_.empty();
  const bool has_gripper_cmd = !gripper_cmd_queue_.empty();

  CusJointCmd joint_cmd_to_use, gripper_cmd_to_use;
  bool use_joint_queue = false, use_gripper_queue = false;

  // Case 1: Both queues have commands
  if (has_joint_cmd && has_gripper_cmd) {
    joint_cmd_to_use = joint_cmd_queue_.front();
    gripper_cmd_to_use = gripper_cmd_queue_.front();

    if (can_fuse(joint_cmd_to_use, gripper_cmd_to_use)) {
      use_joint_queue = use_gripper_queue = true;
    } else {
      LE_LOG_INFO_T(1s) << "FuseCommand: Commands cannot be fused due to "
                           "timestamp difference (has_joint_cmd: "
                        << has_joint_cmd
                        << ", has_gripper_cmd: " << has_gripper_cmd << ")"
                        << std::endl;
      // Pop only the older command when cannot fuse
      if (joint_cmd_to_use.timestamp_ns <= gripper_cmd_to_use.timestamp_ns) {
        joint_cmd_queue_.pop_front();
        LE_LOG_INFO_T(1s)
            << "FuseCommand: Popped older joint command (timestamp: "
            << joint_cmd_to_use.timestamp_ns << ")" << std::endl;
      } else {
        gripper_cmd_queue_.pop_front();
        LE_LOG_INFO_T(1s)
            << "FuseCommand: Popped older gripper command (timestamp: "
            << gripper_cmd_to_use.timestamp_ns << ")" << std::endl;
      }
      return std::nullopt;
    }
  }
  // Case 2: Only joint queue has commands, try to fuse with last gripper cmd
  else if (has_joint_cmd && !has_gripper_cmd) {
    if (!last_gripper_cmd_.empty()) {
      joint_cmd_to_use = joint_cmd_queue_.front();
      gripper_cmd_to_use = last_gripper_cmd_.back();
      use_joint_queue = true;
    } else {
      LE_LOG_INFO_T(1s) << "FuseCommand: No last gripper command available "
                           "for fusion (has_joint_cmd: "
                        << has_joint_cmd
                        << ", has_gripper_cmd: " << has_gripper_cmd << ")"
                        << std::endl;
      return std::nullopt;
    }
  }
  // Case 3: Only gripper queue has commands, try to fuse with last joint cmd
  else if (!has_joint_cmd && has_gripper_cmd) {
    if (!last_joint_cmd_.empty()) {
      joint_cmd_to_use = last_joint_cmd_.back();
      gripper_cmd_to_use = gripper_cmd_queue_.front();
      use_gripper_queue = true;
    } else {
      LE_LOG_INFO_T(1s) << "FuseCommand: No last joint command available for "
                           "fusion (has_joint_cmd: "
                        << has_joint_cmd
                        << ", has_gripper_cmd: " << has_gripper_cmd << ")"
                        << std::endl;
      return std::nullopt;
    }
  } else {
    LE_LOG_INFO_T(1s) << "FuseCommand: Both queues are empty (has_joint_cmd: "
                      << has_joint_cmd
                      << ", has_gripper_cmd: " << has_gripper_cmd << ")"
                      << std::endl;
    return std::nullopt;
  }

  // Create fused command
  auto fused_cmd = create_fused_cmd(joint_cmd_to_use, gripper_cmd_to_use);

  // Update last recorded commands and pop from queues if needed
  if (use_joint_queue) {
    last_joint_cmd_.push_back(joint_cmd_to_use);
    if (last_joint_cmd_.size() > 10) { // Keep only last 10 commands
      last_joint_cmd_.pop_front();
    }
    joint_cmd_queue_.pop_front();
  }
  if (use_gripper_queue) {
    last_gripper_cmd_.push_back(gripper_cmd_to_use);
    if (last_gripper_cmd_.size() > 10) { // Keep only last 10 commands
      last_gripper_cmd_.pop_front();
    }
    gripper_cmd_queue_.pop_front();
  }

  LE_LOG_INFO_T(5s) << "FuseCommand: Successfully fused commands"
                    << " (joint_count: " << joint_cmd_to_use.joints.size()
                    << ", gripper_count: " << gripper_cmd_to_use.joints.size()
                    << ", used_joint_queue: " << use_joint_queue
                    << ", used_gripper_queue: " << use_gripper_queue << ")"
                    << std::endl;

  return fused_cmd;
}

void RobotControlInterface::ControlRobot() {
  if (!robot_communicate_interface_) {
    LE_LOG_ERROR << "RobotCommunicateInterface is not initialized" << std::endl;
    return;
  }

  // Try to get a fused command
  auto fused_cmd = FuseCommand();
  if (fused_cmd.has_value()) {
    // Update joint command in robot communicate interface
    bool success =
        robot_communicate_interface_->UpdateJointCmd(fused_cmd.value());
    if (success) {
      // Publish the motor command
      robot_communicate_interface_->PublishMotorCmd();
      LE_LOG_INFO_T(1s)
          << "ControlRobot: Successfully sent fused command to robot"
          << std::endl;
    } else {
      LE_LOG_ERROR << "ControlRobot: Failed to update joint command"
                   << std::endl;
    }
  } else {
    LE_LOG_INFO_T(5s) << "ControlRobot: No commands available for fusion"
                      << std::endl;
  }
}

void RobotControlInterface::StartControlThread() {
  if (control_thread_running_.load()) {
    LE_LOG_INFO << "Control thread is already running" << std::endl;
    return;
  }

  control_thread_running_.store(true);

  control_thread_ = std::thread([this]() {
    LE_LOG_INFO << "Control robot thread started at 100Hz" << std::endl;

    auto next_wake_time = std::chrono::steady_clock::now();

    while (control_thread_running_.load() && !should_stop_.load()) {
      // Control robot at 100Hz
      ControlRobot();

      // Calculate next wake time to maintain 100Hz frequency
      next_wake_time += kControlPeriod;

      // Sleep until next wake time
      std::this_thread::sleep_until(next_wake_time);

      // If we're running behind, reset the next wake time
      auto current_time = std::chrono::steady_clock::now();
      if (next_wake_time < current_time) {
        next_wake_time = current_time;
        LE_LOG_ERROR_T(1s) << "Control thread running behind schedule"
                           << std::endl;
      }
    }

    LE_LOG_INFO << "Control robot thread stopped" << std::endl;
  });
}

void RobotControlInterface::StopControlThread() {
  if (!control_thread_running_.load()) {
    return;
  }

  LE_LOG_INFO << "Stopping control robot thread..." << std::endl;

  control_thread_running_.store(false);

  if (control_thread_.joinable()) {
    control_thread_.join();
  }

  LE_LOG_INFO << "Control robot thread stopped" << std::endl;
}

JointPositionState RobotControlInterface::GetJointPositionState() const {
  JointPositionState joint_state;

  if (!robot_communicate_interface_) {
    LE_LOG_ERROR << "RobotCommunicateInterface is not initialized" << std::endl;
    return joint_state;
  }

  // Delegate to RobotCommunicateInterface to get joint position state
  return robot_communicate_interface_->GetJointPositionState();
}

} // namespace lerobot_vr_controller
