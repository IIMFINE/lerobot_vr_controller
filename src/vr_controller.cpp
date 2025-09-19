#include "vr_controller.h"
#include "log.h"

#include <chrono>
#include <fstream>
#include <sstream>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace lerobot_vr_controller {

VrTfReceiver::VrTfReceiver(std::shared_ptr<rclcpp::Node> node) : node_(node) {
  // Initialize TF broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

  // Initialize TF buffer and listener
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  // Create separate callback groups for different operations
  calibration_callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  publish_callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  enqueue_callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  joy_callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  joint_state_callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  LE_LOG_INFO << "VrTfReceiver initialized" << std::endl;
}

VrTfReceiver::~VrTfReceiver() {
  ee_to_joint_worker_running_ = false;
  target_ee_pose_queue_cond_.notify_all();
  if (ee_to_joint_worker_.joinable()) {
    ee_to_joint_worker_.join();
  }
}

bool VrTfReceiver::Initialize(const std::string &yaml_file_path,
                              const std::string &urdf_file_path) {
  // Perform any additional initialization steps here
  LE_LOG_INFO << "VrTfReceiver::Initialize() called" << std::endl;

  // Initialize YAML configuration
  yaml_config_path_ = yaml_file_path;
  urdf_file_path_ = urdf_file_path;

  if (!LoadYamlConfig(yaml_config_path_)) {
    LE_LOG_ERROR << "Failed to load YAML configuration from: "
                 << yaml_config_path_ << std::endl;
    return false;
  }

  LE_LOG_INFO << "YAML configuration loaded successfully" << std::endl;

  // Initialize IK solvers after loading configuration
  if (!InitIkSolver()) {
    LE_LOG_ERROR << "Failed to initialize IK solvers" << std::endl;
    return false;
  }

  return true;
}

void VrTfReceiver::Start() {
  // Create joint command publisher for rviz2 visualization
  joint_state_publisher_ =
      node_->create_publisher<sensor_msgs::msg::JointState>(
          "/vr_controller/joint_cmd", rclcpp::QoS(10));

  // Create timer to execute CalibrateVr2GripperTf at 10Hz (100ms interval)
  calibration_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&VrTfReceiver::CalibrateVr2GripperTf, this),
      calibration_callback_group_);

  // Initialize VR to gripper TF publishing timer at 100Hz
  vr_to_gripper_publish_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&VrTfReceiver::Vr2GripperTfPublish, this),
      publish_callback_group_);

  // Add VR to gripper TF enqueue timer at 50Hz (reduced from 200Hz)
  vr_to_gripper_enqueue_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&VrTfReceiver::Vr2GripperTfEnqueue, this),
      enqueue_callback_group_);

  // Create joystick subscriber
  auto qos = rclcpp::QoS(10).best_effort();
  auto joy_sub_options = rclcpp::SubscriptionOptions();
  joy_sub_options.callback_group = joy_callback_group_;
  joy_subscriber_ = node_->create_subscription<sensor_msgs::msg::Joy>(
      "/vr/controller_right/joy", qos,
      std::bind(&VrTfReceiver::JoystickCallback, this, std::placeholders::_1),
      joy_sub_options);

  // Subscribe to /joint_states to keep the latest joint state
  auto joint_state_sub_options = rclcpp::SubscriptionOptions();
  joint_state_sub_options.callback_group = joint_state_callback_group_;
  joint_state_subscriber_ =
      node_->create_subscription<sensor_msgs::msg::JointState>(
          "/joint_states", rclcpp::QoS(50),
          [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            UpdateJointState(msg);
          },
          joint_state_sub_options);

  if (!ee_to_joint_worker_running_) {
    ee_to_joint_worker_running_ = true;
    ee_to_joint_worker_ = std::thread(&VrTfReceiver::EeToJointWorkerLoop, this);
  }
}

bool VrTfReceiver::LoadYamlConfig(const std::string &yaml_file_path) {
  try {
    YAML::Node config = YAML::LoadFile(yaml_file_path);

    if (!config["vr_to_arm_tf"]) {
      LE_LOG_ERROR << "Missing 'vr_to_arm_tf' section in YAML file"
                   << std::endl;
      return false;
    }

    auto tf_config = config["vr_to_arm_tf"];

    // Load world frame configurations
    if (config["gripper_world_frame"]) {
      gripper_world_frame_ = config["gripper_world_frame"].as<std::string>();
      LE_LOG_INFO << "gripper_world_frame: " << gripper_world_frame_
                  << std::endl;
    } else {
      gripper_world_frame_ = "world"; // default fallback
    }

    if (config["vr_world_frame"]) {
      vr_world_frame_ = config["vr_world_frame"].as<std::string>();
      LE_LOG_INFO << "vr_world_frame: " << vr_world_frame_ << std::endl;
    } else {
      vr_world_frame_ = "world"; // default fallback
    }

    LE_LOG_INFO << "Using gripper world frame: " << gripper_world_frame_
                << std::endl;
    LE_LOG_INFO << "Using VR world frame: " << vr_world_frame_ << std::endl;

    // Load IK tolerance configurations
    if (config["ik_tolerances"]) {
      auto tolerance_config = config["ik_tolerances"];

      if (tolerance_config["position_tolerance"]) {
        position_tolerance_ =
            tolerance_config["position_tolerance"].as<double>();
        LE_LOG_INFO << "Loaded position_tolerance: " << position_tolerance_
                    << std::endl;
      } else {
        position_tolerance_ = 0.01; // default value
        LE_LOG_INFO << "Using default position_tolerance: "
                    << position_tolerance_ << std::endl;
      }

      if (tolerance_config["orientation_tolerance"]) {
        orientation_tolerance_ =
            tolerance_config["orientation_tolerance"].as<double>();
        LE_LOG_INFO << "Loaded orientation_tolerance: "
                    << orientation_tolerance_ << std::endl;
      } else {
        orientation_tolerance_ = 0.5; // default value
        LE_LOG_INFO << "Using default orientation_tolerance: "
                    << orientation_tolerance_ << std::endl;
      }
    } else {
      // Use default values if ik_tolerances section is missing
      position_tolerance_ = 0.01;
      orientation_tolerance_ = 0.5;
      LE_LOG_INFO
          << "ik_tolerances section not found, using defaults - Position: "
          << position_tolerance_ << ", Orientation: " << orientation_tolerance_
          << std::endl;
    }

    // Iterate through all key-value pairs in vr_to_arm_tf
    for (auto it = tf_config.begin(); it != tf_config.end(); ++it) {
      std::string gripper_link = it->first.as<std::string>();
      std::string vr_link = it->second.as<std::string>();

      gripper_link_to_vr_map_[gripper_link] = vr_link;

      LE_LOG_INFO << "Loaded mapping: " << vr_link << " -> " << gripper_link
                  << std::endl;
    }

    return true;

  } catch (const YAML::Exception &e) {
    LE_LOG_ERROR << "YAML parsing error: " << e.what() << std::endl;
    return false;
  } catch (const std::exception &e) {
    LE_LOG_ERROR << "Error loading YAML config: " << e.what() << std::endl;
    return false;
  }
}

void VrTfReceiver::CalibrateVr2GripperTf() {
  if (!tf_buffer_ || !tf_listener_) {
    LE_LOG_ERROR << "TF buffer or listener not initialized" << std::endl;
    return;
  }

  if (!should_calibrate_) {
    return;
  }

  calibrated_flag_ = false;

  {
    std::unique_lock<std::shared_mutex> lock(vr_base_link_dummy_tf_mutex_);
    vr_base_link_dummy_tf_.clear();
  }

  for (const auto &[gripper_link, vr_frame] : gripper_link_to_vr_map_) {
    try {
      // Check if both frames exist in TF tree
      // Get transform from gripper world frame to gripper_link
      geometry_msgs::msg::TransformStamped world_to_gripper;
      world_to_gripper = tf_buffer_->lookupTransform(
          gripper_world_frame_, gripper_link, tf2::TimePointZero);

      // Get transform from VR world frame to vr_frame
      geometry_msgs::msg::TransformStamped world_to_vr;
      world_to_vr = tf_buffer_->lookupTransform(vr_world_frame_, vr_frame,
                                                tf2::TimePointZero);

      // Convert to tf2 transforms for easier computation
      tf2::Transform tf_world_to_gripper;
      tf2::Transform tf_world_to_vr;

      tf2::fromMsg(world_to_gripper.transform, tf_world_to_gripper);
      tf2::fromMsg(world_to_vr.transform, tf_world_to_vr);

      // Compute and store rotation from gripper to VR for future use
      tf2::Quaternion gripper_q = tf_world_to_gripper.getRotation();
      tf2::Quaternion vr_q = tf_world_to_vr.getRotation();
      tf2::Quaternion vr_to_gripper_q = vr_q.inverse() * gripper_q;
      vr_wrist_to_gripper_rot_[gripper_link] = vr_to_gripper_q;

      // Link: vr_world_frame -> vr_base_link_dummy -> vr_gripper_dummy
      // To tf to real arm is: vr_base_link_dummy -> vr_gripper_dummy
      tf2::Transform tf_vr_gripper_dummy =
          tf2::Transform(gripper_q, tf_world_to_vr.getOrigin());

      tf2::Transform vr_base_link_dummy =
          tf_vr_gripper_dummy * tf_world_to_gripper.inverse();

      // Convert back to TransformStamped message
      geometry_msgs::msg::TransformStamped vr_base_link_dummy_msg;
      vr_base_link_dummy_msg.header.frame_id = vr_frame;
      vr_base_link_dummy_msg.child_frame_id = vr_frame + kVrBaseLinkDummySuffix;
      vr_base_link_dummy_msg.header.stamp = node_->now();
      vr_base_link_dummy_msg.transform = tf2::toMsg(vr_base_link_dummy);

      // Store the transformation with exclusive lock
      {
        std::unique_lock<std::shared_mutex> lock(vr_base_link_dummy_tf_mutex_);
        vr_base_link_dummy_tf_[gripper_link] = vr_base_link_dummy_msg;
      }

      calibrated_flag_ = true;
      should_calibrate_ = false;

      LE_LOG_INFO << "Successfully computed and published transform from "
                  << vr_frame << " to " << gripper_link << std::endl;

    } catch (const tf2::TransformException &ex) {
      LE_LOG_ERROR << "Failed to get transform for gripper_link: "
                   << gripper_link << ", vr_frame: " << vr_frame
                   << ". Error: " << ex.what() << std::endl;
    }
  }

  if (!vr_base_link_dummy_tf_.empty()) {
    LE_LOG_INFO << "Successfully computed " << vr_base_link_dummy_tf_.size()
                << " VR to gripper transformations" << std::endl;
  }
}

tf2::Transform VrTfReceiver::Vr2GripperTf(const std::string &gripper_link,
                                          const std::string &vr_frame) {
  // Get current VR transform
  geometry_msgs::msg::TransformStamped vr_transform;
  vr_transform = tf_buffer_->lookupTransform(vr_world_frame_, vr_frame,
                                             tf2::TimePointZero);

  // Apply calibration transformation
  geometry_msgs::msg::TransformStamped calibration_transform;
  {
    std::shared_lock<std::shared_mutex> lock(vr_base_link_dummy_tf_mutex_);
    auto it = vr_base_link_dummy_tf_.find(gripper_link);
    if (it == vr_base_link_dummy_tf_.end()) {
      throw std::runtime_error("Calibration data not found for gripper link: " +
                               gripper_link);
    }
    calibration_transform = it->second;
  }

  // Apply the stored calibration transformation using tf2
  tf2::Transform tf_vr_current;
  tf2::Transform tf_vr_base_link_dummy;

  tf2::fromMsg(vr_transform.transform, tf_vr_current);
  tf2::fromMsg(calibration_transform.transform, tf_vr_base_link_dummy);

  tf2::Quaternion tf_vr_rot_to_gripper =
      tf_vr_current.getRotation() * vr_wrist_to_gripper_rot_[gripper_link];

  tf2::Transform tf_vr_rot_correction =
      tf2::Transform(tf_vr_rot_to_gripper, tf_vr_current.getOrigin());

  tf2::Transform tf_vr_to_gripper_cal =
      tf_vr_base_link_dummy.inverse() * tf_vr_rot_correction;

  return tf_vr_to_gripper_cal;
}

void VrTfReceiver::Vr2GripperTfPublish() {
  if (!calibrated_flag_) {
    return;
  }

  for (const auto &[gripper_link, vr_frame] : gripper_link_to_vr_map_) {
    try {
      // Calculate the transformation using the new Vr2GripperTf function
      tf2::Transform tf_vr_to_gripper_cal =
          Vr2GripperTf(gripper_link, vr_frame);

      // Create child frame name with gripper cal suffix
      std::string child_frame = gripper_link + kGripperCalSuffix;

      // Convert back to geometry_msgs and publish
      geometry_msgs::msg::TransformStamped calibrated_transform;
      calibrated_transform.header.stamp = node_->now();
      calibrated_transform.header.frame_id = gripper_world_frame_;
      calibrated_transform.child_frame_id = child_frame;
      calibrated_transform.transform = tf2::toMsg(tf_vr_to_gripper_cal);

      tf_broadcaster_->sendTransform(calibrated_transform);
    } catch (const std::exception &ex) {
      // Silently continue if transform not available
      // 减少频繁的警告日志输出
    }
  }
}

void VrTfReceiver::Vr2GripperTfEnqueue() {
  // Iterate all mapped gripper links and VR frames
  for (const auto &pair : gripper_link_to_vr_map_) {
    const std::string &gripper_link = pair.first;
    const std::string &vr_frame = pair.second;

    try {
      // Compute transform from VR to gripper target in gripper world frame
      tf2::Transform tf = Vr2GripperTf(gripper_link, vr_frame);

      geometry_msgs::msg::TransformStamped ts;
      ts.header.stamp = node_->now();
      ts.header.frame_id = gripper_world_frame_;
      ts.child_frame_id = gripper_link; // target EE pose for this gripper

      const tf2::Vector3 &t = tf.getOrigin();
      ts.transform.translation.x = t.x();
      ts.transform.translation.y = t.y();
      ts.transform.translation.z = t.z();

      tf2::Quaternion q = tf.getRotation();
      ts.transform.rotation.x = q.x();
      ts.transform.rotation.y = q.y();
      ts.transform.rotation.z = q.z();
      ts.transform.rotation.w = q.w();

      // Enqueue result under mutex protection
      {
        std::unique_lock<std::mutex> lock(target_ee_pose_queue_mutex_);
        auto &queue = target_ee_pose_queue_[gripper_link];
        queue.emplace_back(std::move(ts));

        // Limit queue size to prevent memory bloat and reduce processing load
        while (queue.size() > 10) {
          queue.pop_front();
        }
      }
      target_ee_pose_queue_cond_.notify_one();
    } catch (const std::exception &e) {
      // RCLCPP_WARN(node_->get_logger(), "Failed for %s <- %s: %s",
      //             gripper_link.c_str(), vr_frame.c_str(), e.what());
    }
  }
}

void VrTfReceiver::JoystickCallback(
    const sensor_msgs::msg::Joy::SharedPtr msg) {
  // Check if buttons array has at least 6 elements (index 5)
  if (msg->buttons.size() > 5 && msg->buttons[5] != 0) {
    should_calibrate_ = true;
    LE_LOG_INFO << "Calibration triggered by joystick button 5" << std::endl;
  }
}

bool VrTfReceiver::InitIkSolver() {
  // Read URDF file content
  std::ifstream urdf_file(urdf_file_path_);
  if (!urdf_file.is_open()) {
    LE_LOG_ERROR << "Failed to open URDF file: " << urdf_file_path_
                 << std::endl;
    return false;
  }

  std::string urdf_string((std::istreambuf_iterator<char>(urdf_file)),
                          std::istreambuf_iterator<char>());
  urdf_file.close();

  if (urdf_string.empty()) {
    LE_LOG_ERROR << "URDF file is empty: " << urdf_file_path_ << std::endl;
    return false;
  }

  LE_LOG_INFO << "Successfully loaded URDF file: " << urdf_file_path_
              << std::endl;

  // Clear any existing solvers before initializing new ones
  ik_solvers_.clear();

  // Initialize one IK solver for each gripper link
  size_t gripper_count = gripper_link_to_vr_map_.size();
  LE_LOG_INFO << "Initializing " << gripper_count << " IK solvers for "
              << gripper_count << " grippers" << std::endl;

  for (const auto &[gripper_link, vr_frame] : gripper_link_to_vr_map_) {
    auto ik_solver = std::make_unique<SoArm101Kinematics>();

    try {
      // Use gripper_world_frame_ as base_link and gripper_link as tip_link
      if (!ik_solver->Initialize(urdf_string, gripper_world_frame_,
                                 gripper_link)) {
        LE_LOG_ERROR << "Failed to initialize IK solver for gripper link: "
                     << gripper_link << std::endl;
        // Clear all solvers on failure
        ik_solvers_.clear();
        return false;
      }

      // Store the initialized solver
      ik_solvers_[gripper_link] = std::move(ik_solver);

      // Set the tolerances loaded from YAML configuration
      ik_solvers_[gripper_link]->SetTolerances(position_tolerance_,
                                               orientation_tolerance_);

      LE_LOG_INFO << "IK solver " << ik_solvers_.size() << "/" << gripper_count
                  << " initialized successfully for gripper link: "
                  << gripper_link << " with "
                  << ik_solvers_[gripper_link]->GetNumJoints()
                  << " joints (relaxed precision)" << std::endl;

    } catch (const std::exception &e) {
      LE_LOG_ERROR
          << "Exception during IK solver initialization for gripper link "
          << gripper_link << ": " << e.what() << std::endl;
      // Clear all solvers on failure
      ik_solvers_.clear();
      return false;
    }
  }

  if (ik_solvers_.empty()) {
    LE_LOG_ERROR << "No gripper links found in configuration" << std::endl;
    return false;
  }

  // Verify we have exactly the same number of IK solvers as grippers
  if (ik_solvers_.size() != gripper_link_to_vr_map_.size()) {
    LE_LOG_ERROR << "Mismatch: Expected " << gripper_link_to_vr_map_.size()
                 << " IK solvers but got " << ik_solvers_.size() << std::endl;
    ik_solvers_.clear();
    return false;
  }

  LE_LOG_INFO << "Successfully initialized " << ik_solvers_.size()
              << " IK solvers for " << gripper_link_to_vr_map_.size()
              << " grippers" << std::endl;

  // Log all initialized gripper-solver pairs
  for (const auto &[gripper_link, solver] : ik_solvers_) {
    LE_LOG_INFO << "Gripper '" << gripper_link << "' -> IK solver with "
                << solver->GetNumJoints() << " joints" << std::endl;
  }

  return true;
}

bool VrTfReceiver::IkGripperTf(const std::string &gripper_link,
                               const tf2::Transform &target_transform,
                               std::vector<double> &joint_solution,
                               const std::vector<double> &seed_joints) {
  // Find the IK solver for this gripper link
  auto it = ik_solvers_.find(gripper_link);
  if (it == ik_solvers_.end()) {
    LE_LOG_ERROR << "IK solver not found for gripper link: " << gripper_link
                 << std::endl;
    return false;
  }

  auto &ik_solver = it->second;
  if (!ik_solver || !ik_solver->IsInitialized()) {
    LE_LOG_ERROR << "IK solver not initialized for gripper link: "
                 << gripper_link << std::endl;
    return false;
  }

  try {
    // 使用优化的IK求解，不打印调试信息
    if (ik_solver->SolveIK(target_transform, joint_solution, seed_joints)) {
      return true;
    }
    return false;

  } catch (const std::exception &e) {
    // 减少频繁的警告日志输出 - 每秒最多输出一次
    static auto last_warn_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now -
                                                              last_warn_time)
            .count() > 1000) {
      LE_LOG_ERROR << "Exception during IK solving for gripper link "
                   << gripper_link << ": " << e.what() << std::endl;
      last_warn_time = now;
    }
    return false;
  }
}

void VrTfReceiver::EeToJointWorkerLoop() {
  while (ee_to_joint_worker_running_) {
    // Wait until there's work or shutdown
    {
      std::unique_lock<std::mutex> lock(target_ee_pose_queue_mutex_);
      target_ee_pose_queue_cond_.wait(lock, [this] {
        if (!ee_to_joint_worker_running_)
          return true;
        for (const auto &kv : target_ee_pose_queue_) {
          if (!kv.second.empty())
            return true;
        }
        return false;
      });
    }

    if (!ee_to_joint_worker_running_)
      break;

    // Swap queue to a local variable to minimize lock scope
    std::map<std::string, std::deque<geometry_msgs::msg::TransformStamped>>
        local_queue;
    {
      std::unique_lock<std::mutex> lock(target_ee_pose_queue_mutex_);
      std::swap(local_queue, target_ee_pose_queue_);
    }

    // Process local_queue and convert EE targets to joint states
    if (!local_queue.empty()) {
      ControlJointWithEe(local_queue);
    }
  }
}

void VrTfReceiver::ControlJointWithEe(
    const std::map<std::string,
                   std::deque<geometry_msgs::msg::TransformStamped>>
        &local_queue) {
  // Process each gripper link in the local queue
  for (const auto &[gripper_link, pose_queue] : local_queue) {
    if (pose_queue.empty())
      continue;

    // 只处理最新的几个姿态以减少计算负载
    size_t poses_to_process = std::min(pose_queue.size(), size_t(3));
    size_t start_idx = pose_queue.size() - poses_to_process;

    // Get current joint state as seed for IK
    sensor_msgs::msg::JointState current_joint_state = GetLatestJointState();

    // Find the IK solver for this gripper link to use AlignJointStateToIk
    auto ik_it = ik_solvers_.find(gripper_link);
    if (ik_it == ik_solvers_.end()) {
      // 减少频繁的警告日志输出 - 每5秒最多输出一次
      static auto last_warn_time = std::chrono::steady_clock::now();
      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::milliseconds>(now -
                                                                last_warn_time)
              .count() > 5000) {
        LE_LOG_ERROR << "IK solver not found for gripper link: " << gripper_link
                     << std::endl;
        last_warn_time = now;
      }
      continue;
    }

    std::vector<double> seed_joints;
    if (!ik_it->second->AlignJointStateToIk(current_joint_state.name,
                                            current_joint_state.position,
                                            seed_joints)) {
      // Use empty seed joints as fallback
      seed_joints.clear();
    }

    // Process only the most recent poses
    for (size_t i = start_idx; i < pose_queue.size(); ++i) {
      const auto &target_pose_stamped = pose_queue[i];

      // Convert TransformStamped to tf2::Transform
      tf2::Transform target_transform;
      tf2::fromMsg(target_pose_stamped.transform, target_transform);

      // Prepare for IK solution
      std::vector<double> joint_solution;

      // Call IK solver
      if (IkGripperTf(gripper_link, target_transform, joint_solution,
                      seed_joints)) {
        // Update seed for next iteration
        seed_joints = joint_solution;

        // Only publish the final solution to reduce message load
        if (i == pose_queue.size() - 1) {
          PublishJointCmd(gripper_link, joint_solution);
        }
      }
    }
  }
}

void VrTfReceiver::EePoseIktoJointCmd() {
  // TODO: implement conversion from EE targets to joint commands
}

void VrTfReceiver::UpdateJointState(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  // Store the latest joint state
  latest_joint_state_ = *msg;
}

sensor_msgs::msg::JointState VrTfReceiver::GetLatestJointState() const {
  return latest_joint_state_;
}

void VrTfReceiver::PublishJointCmd(const std::string &gripper_link,
                                   const std::vector<double> &joint_solution) {
  if (!joint_state_publisher_) {
    LE_LOG_ERROR << "Joint command publisher not initialized" << std::endl;
    return;
  }

  // Find the IK solver for this gripper link to get joint names
  auto ik_it = ik_solvers_.find(gripper_link);
  if (ik_it == ik_solvers_.end()) {
    LE_LOG_ERROR << "IK solver not found for gripper link: " << gripper_link
                 << std::endl;
    return;
  }

  auto &ik_solver = ik_it->second;
  if (!ik_solver || !ik_solver->IsInitialized()) {
    LE_LOG_ERROR << "IK solver not initialized for gripper link: "
                 << gripper_link << std::endl;
    return;
  }

  // Get joint names from the IK solver
  std::vector<std::string> joint_names = ik_solver->GetJointNames();

  // Verify joint solution size matches joint names size
  if (joint_solution.size() != joint_names.size()) {
    LE_LOG_ERROR << "Joint solution size (" << joint_solution.size()
                 << ") doesn't match joint names size (" << joint_names.size()
                 << ") for gripper: " << gripper_link << std::endl;
    return;
  }

  // Create and populate joint command message
  sensor_msgs::msg::JointState joint_state_msg;
  joint_state_msg.header.stamp = node_->now();
  joint_state_msg.header.frame_id = gripper_world_frame_;

  joint_state_msg.name = joint_names;
  joint_state_msg.position = joint_solution;

  // Set velocities and efforts to zero (we're only interested in positions for
  // visualization)
  joint_state_msg.velocity.resize(joint_solution.size(), 0.0);
  joint_state_msg.effort.resize(joint_solution.size(), 0.0);

  // Publish the joint command
  joint_state_publisher_->publish(joint_state_msg);

  // 减少DEBUG级别的日志输出频率
  static int debug_counter = 0;
  if (++debug_counter % 100 == 0) { // 每100次输出一次
    LE_LOG_INFO << "Published joint command with " << joint_solution.size()
                << " joints for gripper: " << gripper_link << std::endl;
  }
}

} // namespace lerobot_vr_controller
