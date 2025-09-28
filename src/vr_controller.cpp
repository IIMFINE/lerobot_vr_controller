#include "vr_controller.h"
#include "log.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace lerobot_vr_controller {

VrRobotController::VrRobotController(std::shared_ptr<rclcpp::Node> node)
    : node_(node) {
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
  joint_state_publish_callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  // Initialize VR trigger to joint converter
  trigger_converter_ = std::make_unique<vr_controller::VrTriggerJointConvert>();

  LE_LOG_INFO << "Controller initialized" << std::endl;
}

VrRobotController::~VrRobotController() {
  ee_to_joint_worker_running_ = false;
  target_ee_pose_queue_cond_.notify_all();
  if (ee_to_joint_worker_.joinable()) {
    ee_to_joint_worker_.join();
  }

  // Stop robot control interface
  StopRobotControl();
}

bool VrRobotController::Initialize(
    const std::string &yaml_file_path, const std::string &urdf_file_path,
    const std::string &joint_motor_config_file_path,
    const std::string &motor_calibration_file_path,
    const std::string &motor_cmd_topic) {
  // Perform any additional initialization steps here
  LE_LOG_INFO << "Initialize() called" << std::endl;

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

  // Initialize robot control interface
  robot_control_interface_ = std::make_unique<RobotControlInterface>(
      node_, joint_motor_config_file_path, motor_calibration_file_path,
      motor_cmd_topic);

  if (!robot_control_interface_->Initialize()) {
    LE_LOG_ERROR << "Failed to initialize robot control interface" << std::endl;
    return false;
  }

  LE_LOG_INFO << "Robot control interface initialized successfully"
              << std::endl;

  return true;
}

void VrRobotController::Start() {
  // Create joint command publisher for rviz2 visualization
  joint_state_publisher_ =
      node_->create_publisher<sensor_msgs::msg::JointState>(
          kVrControllerJointCmdTopic, rclcpp::QoS(10));

  // Create robot joint states publisher to kJointStatesTopic at 100Hz
  robot_joint_state_publisher_ =
      node_->create_publisher<sensor_msgs::msg::JointState>(kJointStatesTopic,
                                                            rclcpp::QoS(10));

  // Create timer to execute CalibrateVr2GripperTf at 100Hz (100ms interval)
  calibration_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&VrRobotController::CalibrateVr2GripperTf, this),
      calibration_callback_group_);

  // Initialize VR to gripper TF publishing timer at 100Hz
  vr_to_gripper_publish_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&VrRobotController::Vr2GripperTfPublish, this),
      publish_callback_group_);

  // Add VR to gripper TF enqueue timer at 50Hz (reduced from 200Hz)
  vr_to_gripper_enqueue_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&VrRobotController::UpdateVrPose, this),
      enqueue_callback_group_);

  // Create joint state publish timer at 100Hz (10ms interval)
  joint_state_publish_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&VrRobotController::PublishRobotJointStates, this),
      joint_state_publish_callback_group_);

  // Create joystick subscriber
  auto qos = rclcpp::QoS(10).best_effort();
  auto joy_sub_options = rclcpp::SubscriptionOptions();
  joy_sub_options.callback_group = joy_callback_group_;
  std::string joy_topic_name = kVrControllerRightJoyTopic;
  joy_subscriber_ = node_->create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_name, qos,
      [this, joy_topic_name](const sensor_msgs::msg::Joy::SharedPtr msg) {
        JoystickCallback(msg, joy_topic_name);
      },
      joy_sub_options);

  // Subscribe to /joint_states to keep the latest joint state
  auto joint_state_sub_options = rclcpp::SubscriptionOptions();
  joint_state_sub_options.callback_group = joint_state_callback_group_;
  joint_state_subscriber_ =
      node_->create_subscription<sensor_msgs::msg::JointState>(
          kJointStatesTopic, rclcpp::QoS(50),
          [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            UpdateJointState(msg);
          },
          joint_state_sub_options);

  if (!ee_to_joint_worker_running_) {
    ee_to_joint_worker_running_ = true;
    ee_to_joint_worker_ =
        std::thread(&VrRobotController::EeToJointWorkerLoop, this);
  }
}

bool VrRobotController::LoadYamlConfig(const std::string &yaml_file_path) {
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

    // Load kinematics solver configurations
    if (config["kinematics"]) {
      auto kinematics_config = config["kinematics"];

      kinematics_solver_type_ =
          kinematics_config["solver_type"].as<std::string>("xlerobot");
      kinematics_timeout_ = kinematics_config["timeout"].as<double>(0.005);

      LE_LOG_INFO << "Kinematics solver configuration:" << std::endl;
      LE_LOG_INFO << "  - Solver type: " << kinematics_solver_type_
                  << std::endl;
      LE_LOG_INFO << "  - Timeout: " << kinematics_timeout_ << " seconds"
                  << std::endl;
    } else {
      // Use default values if kinematics section is missing
      kinematics_solver_type_ = "xlerobot";
      kinematics_timeout_ = 0.005;
      LE_LOG_INFO << "Kinematics section not found, using defaults:"
                  << std::endl;
      LE_LOG_INFO << "  - Solver type: " << kinematics_solver_type_
                  << std::endl;
      LE_LOG_INFO << "  - Timeout: " << kinematics_timeout_ << " seconds"
                  << std::endl;
    }

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

      if (tolerance_config["xy_max_reach"]) {
        xy_max_reach_ = tolerance_config["xy_max_reach"].as<double>();
        LE_LOG_INFO << "Loaded xy_max_reach: " << xy_max_reach_ << std::endl;
      } else {
        xy_max_reach_ = 0.5; // default value
        LE_LOG_INFO << "Using default xy_max_reach: " << xy_max_reach_
                    << std::endl;
      }
    } else {
      // Use default values if ik_tolerances section is missing
      position_tolerance_ = 0.01;
      orientation_tolerance_ = 0.5;
      xy_max_reach_ = 0.5;
      LE_LOG_INFO
          << "ik_tolerances section not found, using defaults - Position: "
          << position_tolerance_ << ", Orientation: " << orientation_tolerance_
          << ", XY Max Reach: " << xy_max_reach_ << std::endl;
    }

    // Load joint filter configurations
    // Example YAML configuration:
    // joint_filter:
    //   alpha: 0.3  # Smoothing factor (0.0 < alpha <= 1.0)
    //               # Lower values = stronger filtering, slower response
    //               # Higher values = weaker filtering, faster response
    if (config["joint_filter"]) {
      auto filter_config = config["joint_filter"];

      if (filter_config["alpha"]) {
        filter_alpha_ = filter_config["alpha"].as<double>();
        // 验证alpha值范围
        if (filter_alpha_ <= 0.0 || filter_alpha_ > 1.0) {
          LE_LOG_ERROR << "Invalid filter alpha value: " << filter_alpha_
                       << ", using default 0.3" << std::endl;
          filter_alpha_ = 0.3;
        }
        LE_LOG_INFO << "Loaded joint filter alpha: " << filter_alpha_
                    << std::endl;
      } else {
        filter_alpha_ = 0.3; // default value
        LE_LOG_INFO << "Using default joint filter alpha: " << filter_alpha_
                    << std::endl;
      }
    } else {
      // Use default value if joint_filter section is missing
      filter_alpha_ = 0.3;
      LE_LOG_INFO << "joint_filter section not found, using default alpha: "
                  << filter_alpha_ << std::endl;
    }

    // Load axes fine tune configurations (supports top-level or joy_config)
    YAML::Node axes_config;
    if (config["joy_config"] && config["joy_config"]["axes_config"]) {
      axes_config = config["joy_config"]["axes_config"];
    }

    if (axes_config) {

      if (axes_config["z_advance_scale"]) {
        z_advance_scale_ = axes_config["z_advance_scale"].as<double>();
        LE_LOG_INFO << "Loaded z_advance_scale: " << z_advance_scale_
                    << std::endl;
      } else {
        z_advance_scale_ = 0.01;
        LE_LOG_INFO << "Using default z_advance_scale: " << z_advance_scale_
                    << std::endl;
      }

      if (axes_config["z_clockwise_rotate_scale"]) {
        z_clockwise_rotate_scale_ =
            axes_config["z_clockwise_rotate_scale"].as<double>();
        LE_LOG_INFO << "Loaded z_clockwise_rotate_scale: "
                    << z_clockwise_rotate_scale_ << std::endl;
      } else {
        z_clockwise_rotate_scale_ = 0.08;
        LE_LOG_INFO << "Using default z_clockwise_rotate_scale: "
                    << z_clockwise_rotate_scale_ << std::endl;
      }
    } else {
      z_advance_scale_ = 0.01;
      z_clockwise_rotate_scale_ = 0.08;
      LE_LOG_INFO << "axes_config section not found, using defaults - "
                  << "z_advance_scale: " << z_advance_scale_
                  << ", z_clockwise_rotate_scale: " << z_clockwise_rotate_scale_
                  << std::endl;
    }

    // Configure VR trigger to joint converter
    double trigger_scale = 0.0068; // Default value
    // Parse joy_config section for trigger configurations and joint mappings
    if (config["joy_config"]) {
      auto joy_config = config["joy_config"];

      // Parse trigger_config section within joy_config
      if (joy_config["trigger_config"]) {
        auto trigger_config = joy_config["trigger_config"];

        // Update trigger scale if specified in trigger_config
        if (trigger_config["trigger_to_gripper_scale"]) {
          trigger_scale =
              trigger_config["trigger_to_gripper_scale"].as<double>();
          LE_LOG_INFO << "Updated trigger_to_gripper_scale from "
                         "joy_config/trigger_config: "
                      << trigger_scale << std::endl;
        }

        // Parse joint_mappings
        if (trigger_config["joint_mappings"]) {
          auto joint_mappings = trigger_config["joint_mappings"];
          for (auto it = joint_mappings.begin(); it != joint_mappings.end();
               ++it) {
            std::string joint_name = it->first.as<std::string>();
            std::string vr_topic = it->second.as<std::string>();

            joint_to_vr_topic_map_[joint_name] = vr_topic;

            LE_LOG_INFO << "Loaded joint mapping: " << joint_name << " -> "
                        << vr_topic << std::endl;
          }
        }
      }
    }

    // Also configure trigger converter for any joints specified in
    // joint_mappings
    for (const auto &[joint_name, vr_topic] : joint_to_vr_topic_map_) {
      vr_controller::TriggerJointConfig joint_config(
          joint_name,   // joint_name
          0.0,          // default_joint_position
          0.0,          // trigger_default_position
          trigger_scale // trigger_to_gripper_scale
      );

      trigger_converter_->AddJointConfig(joint_config);
      LE_LOG_INFO << "Configured trigger converter for mapped joint: "
                  << joint_name << " with scale: " << trigger_scale
                  << std::endl;
    }

    // Load the single gripper link mapping from vr_to_arm_tf
    // Expect only one mapping in the configuration
    if (tf_config.size() != 1) {
      LE_LOG_ERROR << "Expected exactly one mapping in vr_to_arm_tf, got "
                   << tf_config.size() << std::endl;
      return false;
    }

    auto it = tf_config.begin();
    tip_link_ = it->first.as<std::string>();
    vr_frame_ = it->second.as<std::string>();

    LE_LOG_INFO << "Loaded single mapping: " << vr_frame_ << " -> " << tip_link_
                << std::endl;

    // Parse home_pose configuration
    if (config["joint_pose"] && config["joint_pose"]["home_pose"]) {
      auto home_pose_config = config["joint_pose"]["home_pose"];

      if (home_pose_config["joint_name"] && home_pose_config["position"]) {
        auto joint_names =
            home_pose_config["joint_name"].as<std::vector<std::string>>();
        auto positions = home_pose_config["position"].as<std::vector<double>>();

        if (joint_names.size() == positions.size()) {
          std::vector<std::pair<std::string, double>> joint_position_pairs;
          for (size_t i = 0; i < joint_names.size(); ++i) {
            joint_position_pairs.emplace_back(joint_names[i], positions[i]);
          }

          home_pose_joint_position_ = JointPositionState(joint_position_pairs);

          LE_LOG_INFO << "Loaded home pose configuration with "
                      << joint_names.size() << " joints" << std::endl;
          for (size_t i = 0; i < joint_names.size(); ++i) {
            LE_LOG_INFO << "  " << joint_names[i] << ": " << positions[i]
                        << std::endl;
          }
        } else {
          LE_LOG_ERROR << "Mismatch between joint_name and position array "
                          "sizes in home_pose configuration"
                       << std::endl;
        }
      } else {
        LE_LOG_ERROR
            << "Missing joint_name or position in home_pose configuration"
            << std::endl;
      }
    } else {
      LE_LOG_INFO
          << "No home_pose configuration found, using default (all joints = 0)"
          << std::endl;
    }

    return true;

  } catch (const std::exception &e) {
    LE_LOG_ERROR << "Error loading YAML config: " << e.what() << std::endl;
    return false;
  }
}

void VrRobotController::CalibrateVr2GripperTf() {
  if (!tf_buffer_ || !tf_listener_) {
    LE_LOG_ERROR << "TF buffer or listener not initialized" << std::endl;
    return;
  }

  if (!should_calibrate_) {
    return;
  }

  if (control_robot_flag_) {
    return;
  }

  {
    std::unique_lock<std::shared_mutex> lock(vr_base_link_dummy_tf_mutex_);
    vr_base_link_dummy_tf_ = geometry_msgs::msg::TransformStamped();
  }

  try {
    // Check if both frames exist in TF tree
    // Get transform from gripper world frame to tip_link_
    geometry_msgs::msg::TransformStamped world_to_gripper;
    world_to_gripper = tf_buffer_->lookupTransform(gripper_world_frame_,
                                                   tip_link_, tf2::TimePointZero);

    // Get transform from VR world frame to vr_frame_
    geometry_msgs::msg::TransformStamped world_to_vr;
    world_to_vr = tf_buffer_->lookupTransform(vr_world_frame_, vr_frame_,
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
    vr_wrist_to_gripper_rot_ = vr_to_gripper_q;

    // Link: vr_world_frame -> vr_base_link_dummy -> vr_gripper_dummy
    // To tf to real arm is: vr_base_link_dummy -> vr_gripper_dummy
    tf2::Transform tf_vr_gripper_dummy =
        tf2::Transform(gripper_q, tf_world_to_vr.getOrigin());

    tf2::Transform vr_base_link_dummy =
        tf_vr_gripper_dummy * tf_world_to_gripper.inverse();

    // Convert back to TransformStamped message
    geometry_msgs::msg::TransformStamped vr_base_link_dummy_msg;
    vr_base_link_dummy_msg.header.frame_id = vr_frame_;
    vr_base_link_dummy_msg.child_frame_id = vr_frame_ + kVrBaseLinkDummySuffix;
    vr_base_link_dummy_msg.header.stamp = node_->now();
    vr_base_link_dummy_msg.transform = tf2::toMsg(vr_base_link_dummy);

    // Store the transformation with exclusive lock
    {
      std::unique_lock<std::shared_mutex> lock(vr_base_link_dummy_tf_mutex_);
      vr_base_link_dummy_tf_ = vr_base_link_dummy_msg;
    }

    calibrated_flag_ = true;
    should_calibrate_ = false;

    LE_LOG_INFO << "Successfully computed and published transform from "
                << vr_frame_ << " to " << tip_link_ << std::endl;

  } catch (const tf2::TransformException &ex) {
    LE_LOG_ERROR_T(1s) << "Failed to get transform for tip_link: " << tip_link_
                       << ", vr_frame: " << vr_frame_ << ". Error: " << ex.what()
                       << std::endl;
  }
}

tf2::Transform VrRobotController::Vr2GripperTf(const std::string &vr_frame) {
  // Get current VR transform
  geometry_msgs::msg::TransformStamped vr_transform;
  vr_transform = tf_buffer_->lookupTransform(vr_world_frame_, vr_frame,
                                             tf2::TimePointZero);

  // Apply calibration transformation
  geometry_msgs::msg::TransformStamped calibration_transform;
  {
    std::shared_lock<std::shared_mutex> lock(vr_base_link_dummy_tf_mutex_);
    calibration_transform = vr_base_link_dummy_tf_;
  }

  // Apply the stored calibration transformation using tf2
  tf2::Transform tf_vr_current;
  tf2::Transform tf_vr_base_link_dummy;

  tf2::fromMsg(vr_transform.transform, tf_vr_current);
  tf2::fromMsg(calibration_transform.transform, tf_vr_base_link_dummy);

  tf2::Quaternion tf_vr_rot_to_gripper =
      tf_vr_current.getRotation() * vr_wrist_to_gripper_rot_;

  tf2::Transform tf_vr_rot_correction =
      tf2::Transform(tf_vr_rot_to_gripper, tf_vr_current.getOrigin());

  tf2::Transform tf_vr_to_gripper_cal =
      tf_vr_base_link_dummy.inverse() * tf_vr_rot_correction;

  return tf_vr_to_gripper_cal;
}

void VrRobotController::Vr2GripperTfPublish() {
  if (!calibrated_flag_) {
    return;
  }

  try {
    // Calculate the transformation using the new Vr2GripperTf function
    tf2::Transform tf_vr_to_gripper_cal = Vr2GripperTf(vr_frame_);

    // Create child frame name with gripper cal suffix
    std::string child_frame = tip_link_ + kGripperCalSuffix;

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

void VrRobotController::UpdateVrPose() {
  try {
    // Compute transform from VR to gripper target in gripper world frame
    tf2::Transform tf = Vr2GripperTf(vr_frame_);

    geometry_msgs::msg::TransformStamped ts;
    ts.header.stamp = node_->now();
    ts.header.frame_id = gripper_world_frame_;
    ts.child_frame_id = tip_link_; // target EE pose for this gripper

    const tf2::Vector3 &t = tf.getOrigin();
    ts.transform.translation.x = t.x();
    ts.transform.translation.y = t.y();
    ts.transform.translation.z = t.z();

    tf2::Quaternion q = tf.getRotation();
    ts.transform.rotation.x = q.x();
    ts.transform.rotation.y = q.y();
    ts.transform.rotation.z = q.z();
    ts.transform.rotation.w = q.w();

    // Enqueue target VR pose
    TargetVrPoseEnqueue(std::move(ts));
  } catch (const std::exception &e) {
    LE_LOG_ERROR_T(1s) << "Exception for tip_link: " << tip_link_
                       << ", vr_frame: " << vr_frame_ << ". Error: " << e.what()
                       << std::endl;
  }
}

void VrRobotController::JoystickCallback(
    const sensor_msgs::msg::Joy::SharedPtr msg, const std::string &topic_name) {
  // VR controller button indices
  constexpr const int kTriggerButton = 0;
  constexpr const int kSideTriggerButton = 1;
  constexpr const int kBButton = 5;

  // VR controller axes indices
  constexpr const int kZClockwiseRotateAxis = 2;
  constexpr const int kZAdvanceAxis = 3;

  constexpr const int kSideTriggerThreshold = 200;

  // Control robot_control_interface_ based on side trigger button value
  if (msg->buttons.size() > kSideTriggerButton && robot_control_interface_) {
    int side_trigger_value = msg->buttons[kSideTriggerButton];
    if (side_trigger_value > kSideTriggerThreshold) {
      StartRobotControl();
      LE_LOG_INFO_T(5s) << "Robot control triggered by side trigger: "
                        << side_trigger_value << std::endl;
    } else {
      StopRobotControl();
      LE_LOG_INFO_T(5s) << "Robot control stopped by side trigger: "
                        << side_trigger_value << std::endl;
    }
  }

  // Check if buttons array has at least 6 elements (B button index)
  if (msg->buttons.size() > kBButton && msg->buttons[kBButton] != 0) {
    should_calibrate_ = true;
    {
      std::unique_lock<std::shared_mutex> lock(ee_pose_fine_tune_mutex_);
      ee_pose_fine_tune_.z_advance_ = 0.0;
      ee_pose_fine_tune_.z_clockwise_rotate_ = 0.0;
    }
    StartRobotControl();
    MoveToHomePose();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    StopRobotControl();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    LE_LOG_INFO_T(1s) << "Moving to home pose triggered by joystick B button"
                      << std::endl;
    LE_LOG_INFO_T(1s) << "Calibration triggered by joystick B button"
                      << std::endl;
  }

  // Handle trigger input for gripper control based on joint_mappings
  // configuration
  if (trigger_converter_) {
    // Find which joint(s) are mapped to this VR topic
    for (const auto &[joint_name, vr_topic] : joint_to_vr_topic_map_) {
      if (vr_topic == topic_name) {
        // This joint is controlled by the current VR topic
        double trigger_value = 0.0;

        // Extract trigger value from the joystick message
        if (msg->buttons.size() > kTriggerButton) {
          trigger_value = static_cast<double>(msg->buttons[kTriggerButton]);
        }

        // Convert trigger value to gripper joint position
        auto gripper_position = ConvertTriggerJointPosition(trigger_value);

        // Convert trigger value to CusJointCmd and send to control robot
        // gripper
        LE_LOG_INFO_T(1s) << "Mapped joint: " << joint_name
                          << ", VR topic: " << vr_topic
                          << ", Trigger value: " << trigger_value
                          << ", Gripper position: " << gripper_position
                          << std::endl;
        auto gripper_cmd = Convert2CusJointCmd(joint_name, trigger_value);
        GripperCmdEnqueue(gripper_cmd);
      }
    }
  }

  // Handle axes input for EE pose fine tune
  if (msg->axes.size() > kZAdvanceAxis) {
    std::unique_lock<std::shared_mutex> lock(ee_pose_fine_tune_mutex_);

    double z_advance_delta = msg->axes[kZAdvanceAxis] * z_advance_scale_;

    // axes[kZAdvanceAxis] -> z_advance (累积增加/减少)
    ee_pose_fine_tune_.z_advance_ += z_advance_delta;

    // axes[kZClockwiseRotateAxis] -> z_clockwise_rotate (累积增加/减少)
    ee_pose_fine_tune_.z_clockwise_rotate_ +=
        msg->axes[kZClockwiseRotateAxis] * z_clockwise_rotate_scale_;

    LE_LOG_INFO_T(2s) << "EE pose fine tune - Z advance: "
                      << ee_pose_fine_tune_.z_advance_
                      << ", Z clockwise rotate: "
                      << ee_pose_fine_tune_.z_clockwise_rotate_ << std::endl;
  }
}

bool VrRobotController::InitIkSolver() {
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

  // Initialize IK solver using factory pattern
  ik_solver_ = KinematicsFactory::CreateKinematics(kinematics_solver_type_);
  if (!ik_solver_) {
    LE_LOG_ERROR << "Failed to create IK solver of type: "
                 << kinematics_solver_type_ << std::endl;
    return false;
  }

  try {
    // Use gripper_world_frame_ as base_link and tip_link_ as tip_link
    if (!ik_solver_->Initialize(urdf_string, gripper_world_frame_, tip_link_,
                                kinematics_timeout_)) {
      LE_LOG_ERROR << "Failed to initialize IK solver for tip link: "
                   << tip_link_ << std::endl;
      ik_solver_.reset();
      return false;
    }

    // Set the tolerances loaded from YAML configuration
    ik_solver_->SetTolerances(position_tolerance_, orientation_tolerance_);

    LE_LOG_INFO << "IK solver (type: " << kinematics_solver_type_
                << ") initialized successfully"
                << " - Base: " << gripper_world_frame_ << ", Tip: " << tip_link_ << " with "
                << ik_solver_->GetNumJoints() << " joints" << std::endl;

  } catch (const std::exception &e) {
    LE_LOG_ERROR << "Exception during IK solver initialization (type: "
                 << kinematics_solver_type_ << "): " << e.what() << std::endl;
    ik_solver_.reset();
    return false;
  }

  if (!ik_solver_) {
    LE_LOG_ERROR << "Failed to initialize IK solver (type: "
                 << kinematics_solver_type_ << ")" << std::endl;
    return false;
  }

  LE_LOG_INFO << "Successfully initialized IK solver (type: "
              << kinematics_solver_type_ << ")" << std::endl;

  // Initialize joint position filters after IK solver is ready
  InitJointFilters();

  return true;
}

void VrRobotController::InitJointFilters() {
  // Clear any existing filters
  joint_filters_.clear();

  // Use configured filter alpha value
  constexpr double kDefaultInitialValue = 0.0;

  // Initialize filters for IK-controlled joints
  // Note: These filters are only used for joints controlled through IK solver,
  // not for trigger-controlled joints (gripper joints)
  if (!ik_solver_ || !ik_solver_->IsInitialized()) {
    LE_LOG_ERROR << "IK solver not initialized, cannot create joint filters"
                 << std::endl;
    return;
  }

  // Get joint names for IK-controlled joints only
  const auto joint_names = ik_solver_->GetJointNames();

  // Initialize filters for each IK-controlled joint
  for (const auto &joint_name : joint_names) {
    joint_filters_[joint_name] = std::make_unique<JointPositionFilter>(
        filter_alpha_, kDefaultInitialValue);
  }

  LE_LOG_INFO << "Initialized " << joint_names.size()
              << " joint filters (alpha=" << filter_alpha_
              << ") for IK-controlled joints" << std::endl;
}

bool VrRobotController::IkGripperTf(const tf2::Transform &target_transform,
                                    std::vector<double> &joint_solution,
                                    const std::vector<double> &seed_joints) {
  if (!ik_solver_ || !ik_solver_->IsInitialized()) {
    LE_LOG_ERROR << "IK solver not initialized for tip link: " << tip_link_
                 << std::endl;
    return false;
  }

  try {
    // 使用优化的IK求解，不打印调试信息
    if (ik_solver_->SolveIK(target_transform, joint_solution, seed_joints)) {
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
      LE_LOG_ERROR << "Exception during IK solving for tip link " << tip_link_
                   << ": " << e.what() << std::endl;
      last_warn_time = now;
    }
    return false;
  }
}

tf2::Transform
VrRobotController::LimitTargetTf(const tf2::Transform &target_transform) const {
  tf2::Vector3 position = target_transform.getOrigin();
  tf2::Quaternion rotation = target_transform.getRotation();

  // Apply xy reach limit constraint
  double xy_distance =
      std::sqrt(position.x() * position.x() + position.y() * position.y());

  if (xy_distance > xy_max_reach_) {
    // Scale down the xy position to fit within the reach limit
    double scale_factor = xy_max_reach_ / xy_distance;
    position.setX(position.x() * scale_factor);
    position.setY(position.y() * scale_factor);

    LE_LOG_INFO << "Limited xy reach from " << xy_distance << " to "
                << xy_max_reach_ << std::endl;
  }

  // Return the limited transform
  tf2::Transform limited_transform;
  limited_transform.setOrigin(position);
  limited_transform.setRotation(rotation);
  return limited_transform;
}

tf2::Transform VrRobotController::NormalizeS101GripperTf(
    const tf2::Transform &target_transform) {
  tf2::Vector3 position = target_transform.getOrigin();
  tf2::Quaternion q = target_transform.getRotation();

  constexpr double kEps = 1e-12;
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kTwoPi = 2.0 * kPi;

  // 若末端位姿位于世界Z轴上(x=y=0)，-Z射线天然与世界Z轴相交
  if (std::abs(position.x()) < 1e-9 && std::abs(position.y()) < 1e-9) {
    q.normalize();
    tf2::Transform out;
    out.setOrigin(position);
    out.setRotation(q);
    return out;
  }

  // 目标：使工具坐标系的 -Z 方向在 XY 平面上的投影与指向原点的径向向量对齐，
  // 从而保证沿 -Z 的射线与世界Z轴相交。
  auto normalize_angle = [kPi, kTwoPi](double angle) {
    while (angle > kPi)
      angle -= kTwoPi;
    while (angle < -kPi)
      angle += kTwoPi;
    return angle;
  };

  tf2::Matrix3x3 rot_m(q);
  // 当前工具坐标系 -Z 在世界系方向
  tf2::Vector3 neg_z = -(rot_m * tf2::Vector3(0.0, 0.0, 1.0));

  // 指向世界Z轴(原点在XY平面投影)的径向单位向量
  tf2::Vector3 radial_dir(-position.x(), -position.y(), 0.0);
  radial_dir.normalize();

  // 若 -Z 的XY投影过小(与世界Z轴近乎平行)，先绕与径向垂直的轴给予微小倾角
  tf2::Vector3 neg_z_xy(neg_z.x(), neg_z.y(), 0.0);
  if (neg_z_xy.length2() < 1e-16) {
    tf2::Vector3 tilt_axis(radial_dir.y(), -radial_dir.x(), 0.0); // 与径向正交
    if (tilt_axis.length2() > kEps) {
      tilt_axis.normalize();
      const double tilt_angle = 0.08726646259971647; // 5度
      tf2::Quaternion q_tilt;
      q_tilt.setRotation(tilt_axis, tilt_angle);
      q = q_tilt * q; // 世界系左乘
      q.normalize();
      rot_m.setRotation(q);
      neg_z = -(rot_m * tf2::Vector3(0.0, 0.0, 1.0));
      neg_z_xy = tf2::Vector3(neg_z.x(), neg_z.y(), 0.0);
    }
  }

  // 绕世界Z轴的偏航校正，使 -Z 的XY投影与径向向量对齐(指向原点)
  double phi_dir = std::atan2(neg_z_xy.y(), neg_z_xy.x());
  double phi_radial = std::atan2(radial_dir.y(), radial_dir.x());
  double yaw_delta = normalize_angle(phi_radial - phi_dir);

  tf2::Quaternion q_yaw;
  q_yaw.setRPY(0.0, 0.0, yaw_delta);
  q = q_yaw * q; // 世界系左乘
  q.normalize();

  // 再次校验对齐方向，若仍反向(背离原点)，再绕世界Z轴翻转180度
  rot_m.setRotation(q);
  neg_z = -(rot_m * tf2::Vector3(0.0, 0.0, 1.0));
  tf2::Vector3 neg_z_xy2(neg_z.x(), neg_z.y(), 0.0);
  if (neg_z_xy2.length2() > kEps) {
    neg_z_xy2.normalize();
    if (neg_z_xy2.dot(radial_dir) < 0.0) {
      tf2::Quaternion q_flip;
      q_flip.setRPY(0.0, 0.0, kPi);
      q = q_flip * q;
      q.normalize();
    }
  }

  tf2::Transform result;
  result.setOrigin(position);
  result.setRotation(q);
  return result;
}

tf2::Transform VrRobotController::ApplyTargetTfFineTune(
    const tf2::Transform &target_transform) const {
  tf2::Transform result = target_transform;

  // Get fine tune values with thread-safe access
  double z_advance;
  double z_clockwise_rotate;
  {
    std::shared_lock<std::shared_mutex> lock(ee_pose_fine_tune_mutex_);
    z_advance = ee_pose_fine_tune_.z_advance_;
    z_clockwise_rotate = ee_pose_fine_tune_.z_clockwise_rotate_;
  }

  // Skip processing if no fine tuning is needed
  if (std::abs(z_advance) < 1e-9 && std::abs(z_clockwise_rotate) < 1e-9) {
    return result;
  }

  // Apply z-axis clockwise rotation: rotate around the transform's local z-axis
  if (std::abs(z_clockwise_rotate) >= 1e-9) {
    // Create rotation quaternion around local z-axis (clockwise rotation means
    // negative angle)
    tf2::Quaternion z_rotation;
    z_rotation.setRPY(0.0, 0.0, -z_clockwise_rotate);

    // Apply rotation in the transform's local coordinate system
    tf2::Quaternion current_rotation = result.getRotation();
    tf2::Quaternion new_rotation = current_rotation * z_rotation;
    new_rotation.normalize();
    result.setRotation(new_rotation);
  }

  // Apply z-axis advance: move along the transform's local z-axis after
  // rotation
  if (std::abs(z_advance) >= 1e-9) {
    tf2::Vector3 local_z_axis = result.getBasis() * tf2::Vector3(0.0, 0.0, 1.0);
    tf2::Vector3 new_origin = result.getOrigin() + local_z_axis * z_advance;
    result.setOrigin(new_origin);
  }

  return result;
}

void VrRobotController::EeToJointWorkerLoop() {
  while (ee_to_joint_worker_running_) {
    // Wait until there's work or shutdown
    {
      std::unique_lock<std::mutex> lock(target_ee_pose_queue_mutex_);
      target_ee_pose_queue_cond_.wait(lock, [this] {
        if (!ee_to_joint_worker_running_)
          return true;
        return !target_ee_pose_queue_.empty();
      });
    }

    if (!ee_to_joint_worker_running_)
      break;

    // Swap queue to a local variable to minimize lock scope
    std::deque<geometry_msgs::msg::TransformStamped> local_queue;
    {
      std::unique_lock<std::mutex> lock(target_ee_pose_queue_mutex_);
      std::swap(local_queue, target_ee_pose_queue_);
    }

    // Process local_queue and convert EE targets to joint states
    if (!local_queue.empty()) {
      ProcessEePose(local_queue);
    }
  }
}

void VrRobotController::ProcessEePose(
    const std::deque<geometry_msgs::msg::TransformStamped> &local_queue) {
  if (local_queue.empty())
    return;

  size_t start_idx = 0;

  // Get current joint state as seed for IK
  JointPositionState current_joint_state = GetLatestJointState();

  if (!ik_solver_ || !ik_solver_->IsInitialized()) {
    LE_LOG_ERROR_T(5s) << "IK solver not initialized" << std::endl;
    return;
  }

  std::vector<double> seed_joints;

  // Extract joint names and positions from JointPositionState
  std::vector<std::string> joint_names;
  std::vector<double> joint_positions;
  for (const auto &[name, position] : current_joint_state.joint_positions) {
    joint_names.push_back(name);
    joint_positions.push_back(position);
  }

  if (!ik_solver_->AlignJointStateToIk(joint_names, joint_positions,
                                       seed_joints)) {
    // Use empty seed joints as fallback
    seed_joints.clear();
  }

  // Process only the most recent poses
  for (size_t i = start_idx; i < local_queue.size(); ++i) {
    const auto &target_pose_stamped = local_queue[i];

    // Convert TransformStamped to tf2::Transform
    tf2::Transform target_transform;
    tf2::fromMsg(target_pose_stamped.transform, target_transform);

    // Prepare for IK solution
    std::vector<double> joint_solution;

    auto crrected_target_transform = NormalizeS101GripperTf(target_transform);

    crrected_target_transform =
        ApplyTargetTfFineTune(crrected_target_transform);

    // Apply constraints and limits to the target transform after fine tuning
    crrected_target_transform = LimitTargetTf(crrected_target_transform);

    // TODO: delete it Publish crrected_target_transform to TF
    {
      geometry_msgs::msg::TransformStamped test_gripper_tf;
      test_gripper_tf.header.stamp = node_->now();
      test_gripper_tf.header.frame_id = gripper_world_frame_;
      test_gripper_tf.child_frame_id = "test_gripper";
      test_gripper_tf.transform = tf2::toMsg(crrected_target_transform);
      tf_broadcaster_->sendTransform(test_gripper_tf);
    }

    // Call IK solver
    if (!IkGripperTf(crrected_target_transform, joint_solution, seed_joints)) {
      LE_LOG_ERROR_T(5s) << "IK solving failed at queue index: " << i
                         << std::endl;
      continue;
    }

    // Apply joint position filtering to IK solution
    // Note: Only IK-controlled joints are filtered, trigger-controlled joints
    // bypass filtering
    std::vector<double> filtered_joint_solution;
    const auto ik_joint_names = ik_solver_->GetJointNames();
    filtered_joint_solution.reserve(joint_solution.size());

    for (size_t j = 0; j < joint_solution.size(); ++j) {
      if (j < ik_joint_names.size()) {
        const auto &joint_name = ik_joint_names[j];
        if (auto joint_filter_it = joint_filters_.find(joint_name);
            joint_filter_it != joint_filters_.end()) {
          // Apply filtering to this IK-controlled joint
          filtered_joint_solution.push_back(
              joint_filter_it->second->Filter(joint_solution[j]));
          continue;
        }
      }
      filtered_joint_solution.push_back(joint_solution[j]);
    }

    filtered_joint_solution = joint_solution;

    // Update seed for next iteration (use filtered values for smoother
    // trajectory)
    seed_joints = filtered_joint_solution;

    PublishJointCmd(filtered_joint_solution);

    // Send pose to control robot using filtered joint solution
    std::vector<std::string> solver_joint_names = ik_solver_->GetJointNames();
    CusJointCmd joint_cmd =
        Convert2CusJointCmd(solver_joint_names, filtered_joint_solution);
    JointCmdEnqueue(joint_cmd);
  }
}

void VrRobotController::UpdateJointState(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  if (!ik_solver_ || !ik_solver_->IsInitialized()) {
    return;
  }

  // Get joint names from the IK solver
  std::vector<std::string> solver_joint_names = ik_solver_->GetJointNames();

  // Convert sensor_msgs::msg::JointState to JointPositionState
  auto joint_state_opt =
      ConvertJointStateToJointPositionState(msg, solver_joint_names);

  // If conversion was successful, update the latest joint state
  if (joint_state_opt.has_value()) {
    UpdateLatestJointState(joint_state_opt.value());
  }
}

JointPositionState VrRobotController::GetLatestJointState() const {
  // Get joint state directly from robot control interface
  if (!robot_control_interface_) {
    LE_LOG_ERROR << "Robot control interface not initialized" << std::endl;
    return JointPositionState();
  }

  // Get the current joint position state from the robot control interface
  JointPositionState full_joint_state =
      robot_control_interface_->GetJointPositionState();

  return full_joint_state;
}

JointPositionState VrRobotController::GetLatestJointPositionState() const {
  std::shared_lock<std::shared_mutex> lock(latest_joint_state_mutex_);
  return latest_joint_state_;
}

double
VrRobotController::ConvertTriggerJointPosition(double trigger_value) const {
  if (!trigger_converter_) {
    LE_LOG_ERROR << "Trigger converter not initialized" << std::endl;
    return 0.0;
  }

  // Use default gripper joint name
  const std::string gripper_joint_name = kDefaultGripperJointName;

  if (!trigger_converter_->HasJoint(gripper_joint_name)) {
    LE_LOG_ERROR << "Joint not configured in trigger converter: "
                 << gripper_joint_name << std::endl;
    return 0.0;
  }

  return trigger_converter_->ConvertJointPosition(gripper_joint_name,
                                                  trigger_value);
}

CusJointCmd
VrRobotController::Convert2CusJointCmd(const std::string &gripper_joint_name,
                                       double trigger_value) const {
  // Get the converted joint position using existing function
  double joint_position = ConvertTriggerJointPosition(trigger_value);

  // Create joint name-position pair
  std::vector<std::pair<std::string, double>> joints;
  joints.emplace_back(gripper_joint_name, joint_position);

  // Create and return CusJointCmd with current timestamp and joint data
  return CusJointCmd(joints);
}

CusJointCmd VrRobotController::Convert2CusJointCmd(
    const std::vector<std::string> &joint_names,
    const std::vector<double> &joint_positions) const {
  // Validate input sizes match
  if (joint_names.size() != joint_positions.size()) {
    LE_LOG_ERROR << "Joint names size (" << joint_names.size() 
                 << ") doesn't match joint positions size (" << joint_positions.size() << ")"
                 << std::endl;
    return CusJointCmd(); // Return empty command
  }

  // Create vector of joint name-position pairs
  std::vector<std::pair<std::string, double>> joints;
  joints.reserve(joint_names.size());
  
  for (size_t i = 0; i < joint_names.size(); ++i) {
    joints.emplace_back(joint_names[i], joint_positions[i]);
  }

  // Create and return CusJointCmd with current timestamp and joint data
  return CusJointCmd(joints);
}

void VrRobotController::PublishJointCmd(
    const std::vector<double> &joint_solution) {
  if (!joint_state_publisher_) {
    LE_LOG_ERROR << "Joint command publisher not initialized" << std::endl;
    return;
  }

  if (!ik_solver_ || !ik_solver_->IsInitialized()) {
    LE_LOG_ERROR << "IK solver not initialized" << std::endl;
    return;
  }

  // Get joint names from the IK solver
  std::vector<std::string> joint_names = ik_solver_->GetJointNames();

  // Verify joint solution size matches joint names size
  if (joint_solution.size() != joint_names.size()) {
    LE_LOG_ERROR << "Joint solution size (" << joint_solution.size()
                 << ") doesn't match joint names size (" << joint_names.size()
                 << ")" << std::endl;
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
}

void VrRobotController::TargetVrPoseEnqueue(
    geometry_msgs::msg::TransformStamped ts) {

  if (!IsControlRobot()) {
    return;
  }

  // Enqueue result under mutex protection
  {
    std::unique_lock<std::mutex> lock(target_ee_pose_queue_mutex_);
    target_ee_pose_queue_.emplace_back(std::move(ts));

    // Limit queue size to prevent memory bloat and reduce processing load
    while (target_ee_pose_queue_.size() > 10) {
      target_ee_pose_queue_.pop_front();
    }
  }
  target_ee_pose_queue_cond_.notify_one();
}

void VrRobotController::JointCmdEnqueue(const CusJointCmd &joint_cmd) {
  if (!robot_control_interface_) {
    LE_LOG_ERROR << "Robot control interface not initialized" << std::endl;
    return;
  }

  // Enqueue the command to the robot control interface directly
  if (!robot_control_interface_->EnqueueJointCommand(joint_cmd)) {
    LE_LOG_ERROR_T(5s) << "Failed to enqueue joint command" << std::endl;
  }
}

void VrRobotController::GripperCmdEnqueue(const CusJointCmd &gripper_cmd) {
  if (!robot_control_interface_) {
    LE_LOG_ERROR << "Robot control interface not initialized" << std::endl;
    return;
  }

  // Enqueue the command to the robot control interface directly
  if (!robot_control_interface_->EnqueueGripperCommand(gripper_cmd)) {
    LE_LOG_ERROR_T(5s) << "Failed to enqueue gripper command" << std::endl;
  }
}

void VrRobotController::StartRobotControl() {
  if (!robot_control_interface_) {
    LE_LOG_ERROR << "Robot control interface not initialized" << std::endl;
    return;
  }

  if (!control_robot_flag_) {
    robot_control_interface_->Start();
    control_robot_flag_ = true;
    LE_LOG_INFO << "Robot control interface started" << std::endl;
  }
}

void VrRobotController::StopRobotControl() {
  if (!robot_control_interface_) {
    LE_LOG_ERROR << "Robot control interface not initialized" << std::endl;
    return;
  }

  if (control_robot_flag_) {
    robot_control_interface_->Stop();
    control_robot_flag_ = false;
    LE_LOG_INFO << "Robot control interface stopped" << std::endl;
  }
}

bool VrRobotController::MoveToHomePose() {
  if (!ik_solver_) {
    LE_LOG_ERROR << "No IK solver available" << std::endl;
    return false;
  }

  LE_LOG_INFO << "Moving robot to home pose" << std::endl;

  bool all_success = true;

  // Use configured home pose if available
  if (!home_pose_joint_position_.joint_positions.empty()) {
    LE_LOG_INFO << "Using configured home pose with "
                << home_pose_joint_position_.joint_positions.size() << " joints"
                << std::endl;

    if (!ik_solver_) {
      LE_LOG_ERROR << "IK solver is null" << std::endl;
      all_success = false;
    } else {
      // Get joint names from the IK solver
      std::vector<std::string> solver_joint_names = ik_solver_->GetJointNames();
      if (solver_joint_names.empty()) {
        LE_LOG_ERROR << "No joint names found for tip link: " << tip_link_
                     << std::endl;
        all_success = false;
      } else {
        // Find matching joints from home pose configuration
        std::vector<std::string> matching_joint_names;
        std::vector<double> matching_positions;

        for (const std::string &solver_joint : solver_joint_names) {
          bool found = false;
          for (const auto &[home_joint_name, home_position] :
               home_pose_joint_position_.joint_positions) {
            if (solver_joint == home_joint_name) {
              matching_joint_names.push_back(home_joint_name);
              matching_positions.push_back(home_position);
              found = true;
              break;
            }
          }
          if (!found) {
            // Use default 0.0 for joints not specified in home pose
            matching_joint_names.push_back(solver_joint);
            matching_positions.push_back(0.0);
            LE_LOG_INFO << "Joint " << solver_joint
                        << " not found in home pose config, using default 0.0"
                        << std::endl;
          }
        }

        // Convert to CusJointCmd
        CusJointCmd home_cmd =
            Convert2CusJointCmd(matching_joint_names, matching_positions);

        // Enqueue the command
        JointCmdEnqueue(home_cmd);

        LE_LOG_INFO << "Enqueued home pose command with "
                    << matching_joint_names.size() << " joints" << std::endl;
      }
    }

    // Handle gripper joint from home pose configuration
    std::string gripper_joint_name = "gripper";
    double home_gripper_position = 0.0; // default

    // Look for gripper joint in home pose configuration
    for (const auto &[home_joint_name, home_position] :
         home_pose_joint_position_.joint_positions) {
      if (home_joint_name == gripper_joint_name) {
        home_gripper_position = home_position;
        break;
      }
    }

    // Convert to CusJointCmd
    auto gripper_home_cmd =
        Convert2CusJointCmd(gripper_joint_name, home_gripper_position);

    // Enqueue the gripper command
    GripperCmdEnqueue(gripper_home_cmd);

    LE_LOG_INFO << "Enqueued home gripper command for joint: "
                << gripper_joint_name << " (position: " << home_gripper_position
                << ")" << std::endl;

  } else {
    // Fallback to original behavior (all joints = 0) when no home pose is
    // configured
    LE_LOG_INFO << "No home pose configuration available, using default (all "
                   "joints = 0)"
                << std::endl;

    if (!ik_solver_) {
      LE_LOG_ERROR << "IK solver is null" << std::endl;
      all_success = false;
    } else {
      // Get joint names from the IK solver
      std::vector<std::string> joint_names = ik_solver_->GetJointNames();
      if (joint_names.empty()) {
        LE_LOG_ERROR << "No joint names found for tip link: " << tip_link_
                     << std::endl;
        all_success = false;
      } else {
        // Create home position (all joints = 0)
        std::vector<double> home_positions(joint_names.size(), 0.0);

        // Convert to CusJointCmd
        CusJointCmd home_cmd = Convert2CusJointCmd(joint_names, home_positions);

        // Enqueue the command
        JointCmdEnqueue(home_cmd);

        LE_LOG_INFO << "Enqueued home pose command with " << joint_names.size()
                    << " joints" << std::endl;
      }
    }

    // Also move gripper joint to home position (closed state = 0.0)
    // Create home position for gripper joint (closed state)
    std::string gripper_joint_name = "gripper";
    double home_trigger_value = 0.0;

    // Convert to CusJointCmd using the same method as joystick callback
    auto gripper_home_cmd =
        Convert2CusJointCmd(gripper_joint_name, home_trigger_value);

    // Enqueue the gripper command
    GripperCmdEnqueue(gripper_home_cmd);

    LE_LOG_INFO << "Enqueued home gripper command for joint: "
                << gripper_joint_name
                << " (trigger value: " << home_trigger_value << ")"
                << std::endl;
  }

  if (all_success) {
    LE_LOG_INFO
        << "Successfully enqueued home pose commands for robot and gripper"
        << std::endl;
  } else {
    LE_LOG_ERROR << "Failed to enqueue some home pose commands" << std::endl;
  }

  return all_success;
}

sensor_msgs::msg::JointState VrRobotController::ConvertToRosJointState(
    const JointPositionState &joint_position_state) const {
  sensor_msgs::msg::JointState joint_state_msg;
  joint_state_msg.header.stamp =
      rclcpp::Time(joint_position_state.timestamp_ns);

  for (const auto &[joint_name, position] :
       joint_position_state.joint_positions) {
    joint_state_msg.name.push_back(joint_name);
    joint_state_msg.position.push_back(position);
  }

  // Set velocities and efforts to zero (not available in JointPositionState)
  joint_state_msg.velocity.resize(joint_state_msg.name.size(), 0.0);
  joint_state_msg.effort.resize(joint_state_msg.name.size(), 0.0);

  return joint_state_msg;
}

std::optional<JointPositionState>
VrRobotController::ConvertJointStateToJointPositionState(
    const sensor_msgs::msg::JointState::SharedPtr msg,
    const std::vector<std::string> &solver_joint_names) {
  // Check if all joints from the solver are present in the message
  bool all_joints_found = true;
  std::vector<size_t> joint_indices;

  std::for_each(solver_joint_names.begin(), solver_joint_names.end(),
                [&](const std::string &joint_name) {
                  auto it =
                      std::find_if(msg->name.begin(), msg->name.end(),
                                   [&joint_name](const std::string &name) {
                                     return name == joint_name;
                                   });
                  if (it == msg->name.end()) {
                    all_joints_found = false;
                    return;
                  }
                  joint_indices.push_back(std::distance(msg->name.begin(), it));
                });

  // If all joints are found, create JointPositionState
  if (all_joints_found) {
    // Create joint name-position pairs
    std::vector<std::pair<std::string, double>> joint_positions;
    joint_positions.reserve(joint_indices.size());

    for (size_t idx : joint_indices) {
      joint_positions.emplace_back(msg->name[idx], msg->position[idx]);
    }

    // Create JointPositionState with current timestamp
    return JointPositionState(joint_positions);
  }

  // Return std::nullopt if not all joints are found
  return std::nullopt;
}

void VrRobotController::UpdateLatestJointState(
    const JointPositionState &joint_state) {
  // Store the joint position state with thread safety
  {
    std::unique_lock<std::shared_mutex> lock(latest_joint_state_mutex_);
    latest_joint_state_ = joint_state;
  }
}

void VrRobotController::PublishRobotJointStates() {
  if (!robot_joint_state_publisher_) {
    LE_LOG_ERROR << "Robot joint state publisher not initialized" << std::endl;
    return;
  }

  // Get the latest joint state
  JointPositionState joint_position_state = GetLatestJointState();

  if (joint_position_state.joint_positions.empty()) {
    // Skip if no joint state available
    return;
  }

  // Create joint state message
  sensor_msgs::msg::JointState joint_state_msg;
  joint_state_msg.header.stamp = node_->now();
  joint_state_msg.header.frame_id = gripper_world_frame_;

  // Add each joint to the message
  for (const auto &[joint_name, position] :
       joint_position_state.joint_positions) {
    joint_state_msg.name.push_back(joint_name);
    joint_state_msg.position.push_back(position);
    joint_state_msg.velocity.push_back(0.0); // Set velocity to zero
    joint_state_msg.effort.push_back(0.0);   // Set effort to zero
  }

  // Publish the joint state
  robot_joint_state_publisher_->publish(joint_state_msg);
}

} // namespace lerobot_vr_controller
