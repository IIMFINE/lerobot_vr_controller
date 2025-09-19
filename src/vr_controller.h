#ifndef RECEVICE_VR_TF_H_
#define RECEVICE_VR_TF_H_

#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <shared_mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "kinematics.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "yaml-cpp/yaml.h"

namespace lerobot_vr_controller {

// Constants
static constexpr const char *kGripperCalSuffix = "_cal";
static constexpr const char *kVrBaseLinkDummySuffix = "_vr_dummy";

class VrTfReceiver {
public:
  explicit VrTfReceiver(std::shared_ptr<rclcpp::Node> node);
  ~VrTfReceiver();

  // Initialize the VR TF receiver with YAML configuration
  bool Initialize(const std::string &yaml_file_path,
                  const std::string &urdf_file_path);

  // Start receiving VR data
  void Start();

  // Get the latest joint state snapshot for a specific gripper
  sensor_msgs::msg::JointState
  GetLatestJointState(const std::string &gripper_link) const;

private:
  bool InitIkSolver();

  bool IkGripperTf(const std::string &gripper_link,
                   const tf2::Transform &target_transform,
                   std::vector<double> &joint_solution,
                   const std::vector<double> &seed_joints = {});

  tf2::Transform Vr2GripperTf(const std::string &gripper_link,
                              const std::string &vr_frame);

  void CalibrateVr2GripperTf();

  void Vr2GripperTfPublish();

  // Similar to Vr2GripperTfPublish but enqueue target EE poses instead of
  // broadcasting TF
  void Vr2GripperTfEnqueue();

  void JoystickCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

  // Load configuration from YAML
  bool LoadYamlConfig(const std::string &yaml_file_path);

  // Converts queued EE targets to joint commands (stub, to be implemented
  // later)
  void EePoseIktoJointCmd();

  // Control joint with end effector poses from local queue
  void ControlJointWithEe(
      const std::map<std::string,
                     std::deque<geometry_msgs::msg::TransformStamped>>
          &local_queue);

  // Worker loop that watches the target_ee_pose_queue_ and triggers
  // EePoseIktoJointCmd
  void EeToJointWorkerLoop();

  // Callback to update latest joint state
  void UpdateJointState(const sensor_msgs::msg::JointState::SharedPtr msg);

  // Publish joint commands for rviz2 visualization
  void PublishJointCmd(const std::string &gripper_link,
                       const std::vector<double> &joint_solution);

  // Node pointer passed from main
  std::shared_ptr<rclcpp::Node> node_;

  // Callback groups for different operations
  rclcpp::CallbackGroup::SharedPtr calibration_callback_group_;
  rclcpp::CallbackGroup::SharedPtr publish_callback_group_;
  rclcpp::CallbackGroup::SharedPtr enqueue_callback_group_;
  rclcpp::CallbackGroup::SharedPtr joy_callback_group_;
  rclcpp::CallbackGroup::SharedPtr joint_state_callback_group_;

  // TF broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // TF buffer and listener
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // Configuration file path
  std::string yaml_config_path_;

  // URDF file path
  std::string urdf_file_path_;

  std::map<std::string, std::string> gripper_link_to_vr_map_;

  // Configurable world frame names
  std::string gripper_world_frame_;
  std::string vr_world_frame_;

  std::atomic<bool> should_calibrate_{false};
  std::atomic<bool> calibrated_flag_{false};

  // Store VR to gripper transformation matrices
  std::map<std::string, geometry_msgs::msg::TransformStamped>
      vr_base_link_dummy_tf_;

  // Queue of target EE poses per gripper link
  std::map<std::string, std::deque<geometry_msgs::msg::TransformStamped>>
      target_ee_pose_queue_;

  // Shared mutex for thread-safe access to vr_base_link_dummy_tf_
  mutable std::shared_mutex vr_base_link_dummy_tf_mutex_;

  // Mutex + condition variable for target_ee_pose_queue_
  mutable std::mutex target_ee_pose_queue_mutex_;
  std::condition_variable target_ee_pose_queue_cond_;

  std::map<std::string, tf2::Quaternion> vr_wrist_to_gripper_rot_;

  // Timer for calibration at 10Hz
  rclcpp::TimerBase::SharedPtr calibration_timer_;

  // Timer for VR to gripper TF publishing at 100Hz
  rclcpp::TimerBase::SharedPtr vr_to_gripper_publish_timer_;

  // New 100Hz timer for enqueuing VR->gripper targets
  rclcpp::TimerBase::SharedPtr vr_to_gripper_enqueue_timer_;

  // Joystick subscriber
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;

  // Joint states subscriber and latest joint state storage
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_subscriber_;

  std::map<std::string, sensor_msgs::msg::JointState> latest_joint_state_map_;

  // Shared mutex for thread-safe access to latest_joint_state_map_
  mutable std::shared_mutex latest_joint_state_map_mutex_;

  // Joint command publisher for rviz2 visualization
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_publisher_;

  // IK solvers for each gripper link
  std::map<std::string, std::unique_ptr<SoArm101Kinematics>> ik_solvers_;

  // IK tolerance configurations loaded from YAML
  double position_tolerance_;
  double orientation_tolerance_;

  // Dedicated worker thread to process EE targets into joint commands
  std::atomic<bool> ee_to_joint_worker_running_{false};
  std::thread ee_to_joint_worker_;
};

} // namespace lerobot_vr_controller

#endif // RECEVICE_VR_TF_H_
