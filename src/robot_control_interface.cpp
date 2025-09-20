#include "robot_control_interface.h"

namespace lerobot_vr_controller {

RobotControlInterface::~RobotControlInterface() {
  Stop();
  LE_LOG_INFO << "RobotControlInterface destroyed" << std::endl;
}

bool RobotControlInterface::Initialize() {
  LE_LOG_INFO << "Initializing RobotControlInterface..." << std::endl;
  LE_LOG_INFO << "RobotControlInterface initialized successfully" << std::endl;
  return true;
}

void RobotControlInterface::Start() {
  if (is_running_.load()) {
    LE_LOG_INFO << "RobotControlInterface is already running" << std::endl;
    return;
  }

  LE_LOG_INFO << "Starting RobotControlInterface..." << std::endl;

  should_stop_.store(false);
  is_running_.store(true);

  LE_LOG_INFO << "RobotControlInterface started successfully" << std::endl;
}

void RobotControlInterface::Stop() {
  if (!is_running_.load()) {
    return;
  }

  LE_LOG_INFO << "Stopping RobotControlInterface..." << std::endl;

  should_stop_.store(true);
  is_running_.store(false);

  // Notify all waiting threads
  joint_cmd_queue_cond_.notify_all();
  gripper_cmd_queue_cond_.notify_all();

  LE_LOG_INFO << "RobotControlInterface stopped" << std::endl;
}

bool RobotControlInterface::EnqueueJointCommand(
    const std::string &robot_name,
    const sensor_msgs::msg::JointState &joint_cmd) {

  std::unique_lock<std::shared_mutex> lock(joint_cmd_queue_mutex_);

  auto &queue = joint_cmd_queue_[robot_name];
  if (queue.size() >= kMaxQueueSize) {
    LE_LOG_ERROR_T(1s) << "Joint command queue for robot '" << robot_name
                       << "' is full (size: " << queue.size() << ")"
                       << std::endl;
    return false;
  }

  queue.push_back(joint_cmd);
  lock.unlock();

  joint_cmd_queue_cond_.notify_one();
  LE_LOG_INFO_T(1s) << "Enqueued joint command for robot: " << robot_name
                    << " (queue size: " << queue.size() << ")" << std::endl;
  return true;
}

bool RobotControlInterface::EnqueueGripperCommand(
    const std::string &robot_name,
    const sensor_msgs::msg::JointState &gripper_cmd) {

  std::unique_lock<std::shared_mutex> lock(gripper_cmd_queue_mutex_);

  auto &queue = gripper_cmd_queue_[robot_name];
  if (queue.size() >= kMaxQueueSize) {
    LE_LOG_ERROR_T(1s) << "Gripper command queue for robot '" << robot_name
                       << "' is full (size: " << queue.size() << ")"
                       << std::endl;
    return false;
  }

  queue.push_back(gripper_cmd);
  lock.unlock();

  gripper_cmd_queue_cond_.notify_one();
  LE_LOG_INFO_T(1s) << "Enqueued gripper command for robot: " << robot_name
                    << " (queue size: " << queue.size() << ")" << std::endl;
  return true;
}

bool RobotControlInterface::DequeueJointCommand(
    const std::string &robot_name, sensor_msgs::msg::JointState &joint_cmd) {

  std::unique_lock<std::shared_mutex> lock(joint_cmd_queue_mutex_);

  auto it = joint_cmd_queue_.find(robot_name);
  if (it == joint_cmd_queue_.end() || it->second.empty()) {
    return false;
  }

  joint_cmd = std::move(it->second.front());
  it->second.pop_front();

  LE_LOG_INFO_T(1s) << "Dequeued joint command for robot: " << robot_name
                    << " (remaining: " << it->second.size() << ")" << std::endl;
  return true;
}

bool RobotControlInterface::DequeueGripperCommand(
    const std::string &robot_name, sensor_msgs::msg::JointState &gripper_cmd) {

  std::unique_lock<std::shared_mutex> lock(gripper_cmd_queue_mutex_);

  auto it = gripper_cmd_queue_.find(robot_name);
  if (it == gripper_cmd_queue_.end() || it->second.empty()) {
    return false;
  }

  gripper_cmd = std::move(it->second.front());
  it->second.pop_front();

  LE_LOG_INFO_T(1s) << "Dequeued gripper command for robot: " << robot_name
                    << " (remaining: " << it->second.size() << ")" << std::endl;
  return true;
}

size_t RobotControlInterface::GetJointCommandQueueSize(
    const std::string &robot_name) const {
  std::shared_lock<std::shared_mutex> lock(joint_cmd_queue_mutex_);

  auto it = joint_cmd_queue_.find(robot_name);
  return (it != joint_cmd_queue_.end()) ? it->second.size() : 0;
}

size_t RobotControlInterface::GetGripperCommandQueueSize(
    const std::string &robot_name) const {
  std::shared_lock<std::shared_mutex> lock(gripper_cmd_queue_mutex_);

  auto it = gripper_cmd_queue_.find(robot_name);
  return (it != gripper_cmd_queue_.end()) ? it->second.size() : 0;
}

void RobotControlInterface::ClearAllCommands(const std::string &robot_name) {
  {
    std::unique_lock<std::shared_mutex> joint_lock(joint_cmd_queue_mutex_);
    auto it = joint_cmd_queue_.find(robot_name);
    if (it != joint_cmd_queue_.end()) {
      it->second.clear();
    }
  }

  {
    std::unique_lock<std::shared_mutex> gripper_lock(gripper_cmd_queue_mutex_);
    auto it = gripper_cmd_queue_.find(robot_name);
    if (it != gripper_cmd_queue_.end()) {
      it->second.clear();
    }
  }

  LE_LOG_INFO << "Cleared all commands for robot: " << robot_name << std::endl;
}

} // namespace lerobot_vr_controller
