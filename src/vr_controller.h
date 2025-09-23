#ifndef RECEVICE_VR_TF_H_
#define RECEVICE_VR_TF_H_

#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <optional>
#include <shared_mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "joint_position_filter.h"
#include "kinematics.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "vr_trigger_joint_convert.h"
#include "yaml-cpp/yaml.h"

#include "interface_type.h"
#include "log.h"
#include "robot_communicate_interface.h"
#include "robot_control_interface.h"

namespace lerobot_vr_controller {

// Constants
static constexpr const char *kGripperCalSuffix = "_cal";
static constexpr const char *kVrBaseLinkDummySuffix = "_vr_dummy";
static constexpr const char *kDefaultGripperJointName = "gripper";

// Topic name constants
static constexpr const char *kVrControllerJointCmdTopic =
    "/vr_controller/joint_cmd";
static constexpr const char *kVrControllerRightJoyTopic =
    "/vr/controller_right/joy";
static constexpr const char *kJointStatesTopic = "/joint_states";

class VrRobotController {
public:
  explicit VrRobotController(std::shared_ptr<rclcpp::Node> node);
  ~VrRobotController();

  // Initialize the VR TF receiver with YAML configuration
  bool
  Initialize(const std::string &yaml_file_path,
             const std::string &urdf_file_path,
             const std::string &joint_motor_config_file_path,
             const std::string &motor_calibration_file_path,
             const std::string &motor_cmd_topic = "/robot_control/motor_cmd");

  // === Core Control Functions ===
  void Start();

  // === Joint State Functions ===
  // Get the latest joint state snapshot for a specific gripper
  JointPositionState GetLatestJointState(const std::string &gripper_link) const;

  // Get the latest joint position state for a specific gripper (returns
  // JointPositionState)
  JointPositionState
  GetLatestJointPositionState(const std::string &gripper_link) const;

  // Convert JointPositionState to sensor_msgs::msg::JointState
  sensor_msgs::msg::JointState
  ConvertToRosJointState(const JointPositionState &joint_position_state) const;

  // === Conversion Functions ===
  // Convert VR trigger value to gripper joint position
  double ConvertTriggerJointPosition(const std::string &gripper_link,
                                     double trigger_value) const;

  // Convert VR trigger value to CusJointCmd
  CusJointCmd Convert2CusJointCmd(const std::string &gripper_joint_name,
                                  double trigger_value) const;

  // Convert vector of joint names and positions to CusJointCmd
  CusJointCmd Convert2CusJointCmd(const std::vector<std::string> &joint_names,
                                  const std::vector<double> &joint_positions) const;

  // === Robot Control Functions ===
  // Start robot control interface
  void StartRobotControl();

  // Stop robot control interface
  void StopRobotControl();

  // Move all robots to home pose (all joints to 0)
  bool MoveToHomePose();

  bool IsControlRobot() const {
    return control_robot_flag_ && !should_calibrate_.load();
  }

private:
  // === IK Solver Functions ===
  bool InitIkSolver();

  // Initialize joint position filters for all grippers
  void InitJointFilters();

  bool IkGripperTf(const std::string &gripper_link,
                   const tf2::Transform &target_transform,
                   std::vector<double> &joint_solution,
                   const std::vector<double> &seed_joints = {});

  // Normalize S101 gripper transform by setting yaw rotation to 0
  tf2::Transform NormalizeS101GripperTf(const tf2::Transform &target_transform);

  // Control joint with end effector poses from local queue
  void
  CalculateIk(const std::map<std::string,
                             std::deque<geometry_msgs::msg::TransformStamped>>
                  &local_queue);

  // Worker loop that watches the target_ee_pose_queue_ and triggers
  // EePoseIktoJointCmd
  void EeToJointWorkerLoop();

  // === VR Transform Functions ===
  tf2::Transform Vr2GripperTf(const std::string &gripper_link,
                              const std::string &vr_frame);

  void CalibrateVr2GripperTf();

  void Vr2GripperTfPublish();

  // Similar to Vr2GripperTfPublish but enqueue target EE poses instead of
  // broadcasting TF
  void UpdateVrPose();

  // === Callback Functions ===
  void JoystickCallback(const sensor_msgs::msg::Joy::SharedPtr msg, const std::string &topic_name);

  // Callback to update latest joint state
  void UpdateJointState(const sensor_msgs::msg::JointState::SharedPtr msg);

  // === Configuration Functions ===
  // Load configuration from YAML
  bool LoadYamlConfig(const std::string &yaml_file_path);

  // === Joint State Conversion Functions ===
  // Convert sensor_msgs::msg::JointState to JointPositionState for specific
  // gripper
  std::optional<JointPositionState> ConvertJointStateToJointPositionState(
      const sensor_msgs::msg::JointState::SharedPtr msg,
      const std::string &gripper_link,
      const std::vector<std::string> &solver_joint_names);

  // Update JointPositionState to latest_joint_state_map_
  void UpdateLatestJointStateMap(const std::string &gripper_link,
                                 const JointPositionState &joint_state);

  // === Command Queue Functions ===
  // Enqueue target VR pose to target_ee_pose_queue_ with mutex protection
  void TargetVrPoseEnqueue(const std::string &gripper_link,
                           geometry_msgs::msg::TransformStamped ts);

  // Enqueue joint command to joint_cmd_queue_
  void JointCmdEnqueue(const std::string &gripper_link,
                       const CusJointCmd &joint_cmd);

  // Enqueue gripper command to gripper_cmd_queue_
  void GripperCmdEnqueue(const std::string &gripper_link,
                         const CusJointCmd &gripper_cmd);

  // === Publishing Functions ===
  // Publish joint commands for rviz2 visualization
  void PublishJointCmd(const std::string &gripper_link,
                       const std::vector<double> &joint_solution);

  // Publish current robot joint states at 100Hz
  void PublishRobotJointStates();

  // === Core Components ===
  // Node pointer passed from main
  std::shared_ptr<rclcpp::Node> node_;

  // === Callback Groups ===
  // Callback groups for different operations
  rclcpp::CallbackGroup::SharedPtr calibration_callback_group_;
  rclcpp::CallbackGroup::SharedPtr publish_callback_group_;
  rclcpp::CallbackGroup::SharedPtr enqueue_callback_group_;
  rclcpp::CallbackGroup::SharedPtr joy_callback_group_;
  rclcpp::CallbackGroup::SharedPtr joint_state_callback_group_;
  rclcpp::CallbackGroup::SharedPtr joint_state_publish_callback_group_;

  // === TF Components ===
  // TF broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // TF buffer and listener
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // === Configuration Data ===
  // Configuration file path
  std::string yaml_config_path_;

  // URDF file path
  std::string urdf_file_path_;

  std::map<std::string, std::string> gripper_link_to_vr_map_;

  // Configurable world frame names
  std::string gripper_world_frame_;
  std::string vr_world_frame_;

  // IK tolerance configurations loaded from YAML
  double position_tolerance_;
  double orientation_tolerance_;
  double xy_max_reach_;

  // Joint filter configurations loaded from YAML
  double filter_alpha_; // 滤波器平滑因子

  // Mapping from joint name to VR topic for trigger control
  std::map<std::string, std::string> joint_to_vr_topic_map_;

  // === VR Calibration Data ===
  std::atomic<bool> should_calibrate_{false};
  std::atomic<bool> calibrated_flag_{false};

  // Store VR to gripper transformation matrices
  std::map<std::string, geometry_msgs::msg::TransformStamped>
      vr_base_link_dummy_tf_;

  std::map<std::string, tf2::Quaternion> vr_wrist_to_gripper_rot_;

  // Shared mutex for thread-safe access to vr_base_link_dummy_tf_
  mutable std::shared_mutex vr_base_link_dummy_tf_mutex_;

  // === Pose Queue Data ===
  // Queue of target EE poses per gripper link
  std::map<std::string, std::deque<geometry_msgs::msg::TransformStamped>>
      target_ee_pose_queue_;

  // Mutex + condition variable for target_ee_pose_queue_
  mutable std::mutex target_ee_pose_queue_mutex_;
  std::condition_variable target_ee_pose_queue_cond_;

  // === Command Queue Data ===
  std::shared_mutex joint_cmd_queue_mutex_;
  std::condition_variable joint_cmd_queue_cond_;
  std::map<std::string, std::deque<sensor_msgs::msg::JointState>>
      joint_cmd_queue_;

  std::shared_mutex gripper_cmd_queue_mutex_;
  std::condition_variable gripper_cmd_queue_cond_;
  std::map<std::string, std::deque<sensor_msgs::msg::JointState>>
      gripper_cmd_queue_;

  // === Joint State Data ===
  std::map<std::string, JointPositionState> latest_joint_state_map_;

  // Shared mutex for thread-safe access to latest_joint_state_map_
  mutable std::shared_mutex latest_joint_state_map_mutex_;

  // === Timers ===
  // Timer for calibration at 10Hz
  rclcpp::TimerBase::SharedPtr calibration_timer_;

  // Timer for VR to gripper TF publishing at 100Hz
  rclcpp::TimerBase::SharedPtr vr_to_gripper_publish_timer_;

  // New 100Hz timer for enqueuing VR->gripper targets
  rclcpp::TimerBase::SharedPtr vr_to_gripper_enqueue_timer_;

  // Timer for publishing joint states at 100Hz
  rclcpp::TimerBase::SharedPtr joint_state_publish_timer_;

  // === Subscribers ===
  // Joystick subscriber
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;

  // Joint states subscriber and latest joint state storage
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_subscriber_;

  // === Publishers ===
  // Joint command publisher for rviz2 visualization
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_publisher_;

  // Joint states publisher to kJointStatesTopic
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      robot_joint_state_publisher_;

  // === Control Components ===
  // IK solvers for each gripper link
  std::map<std::string, std::unique_ptr<SoArm101Kinematics>> ik_solvers_;

  // Joint position filters for each gripper link and joint
  std::map<std::string,
           std::map<std::string, std::unique_ptr<JointPositionFilter>>>
      joint_filters_;

  // VR trigger to joint converter
  std::unique_ptr<vr_controller::VrTriggerJointConvert> trigger_converter_;

  // Robot control interface for managing joint and gripper commands
  std::unique_ptr<RobotControlInterface> robot_control_interface_;

  // === Worker Threads ===
  // Dedicated worker thread to process EE targets into joint commands
  std::atomic<bool> ee_to_joint_worker_running_{false};
  std::thread ee_to_joint_worker_;

  // === Control Flags ===
  std::atomic<bool> control_robot_flag_{false};

  // === Home Pose Configuration ===
  // Home pose joint positions loaded from YAML configuration
  JointPositionState home_pose_joint_position_;
};

} // namespace lerobot_vr_controller

#endif // RECEVICE_VR_TF_H_
