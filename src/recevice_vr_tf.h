#ifndef RECEVICE_VR_TF_H_
#define RECEVICE_VR_TF_H_

#include <memory>
#include <string>
#include <unordered_map>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
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
  ~VrTfReceiver() = default;

  // Initialize YAML configuration
  bool InitializeYamlConfig(const std::string &yaml_file_path);

  // Start receiving VR data
  void Start();

  tf2::Transform Vr2GripperTf(const std::string &gripper_link,
                              const std::string &vr_frame);

private:
  void CalibrateVr2GripperTf();

  void Vr2GripperTfPublish();

  void JoystickCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

  // Load configuration from YAML
  bool LoadYamlConfig(const std::string &yaml_file_path);

  // Node pointer passed from main
  std::shared_ptr<rclcpp::Node> node_;

  // TF broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // TF buffer and listener
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // Configuration file path
  std::string yaml_config_path_;

  std::map<std::string, std::string> gripper_link_to_vr_map_;
  std::atomic<bool> should_calibrate_{false};
  std::atomic<bool> calibrated_flag_{false};

  // Store VR to gripper transformation matrices
  std::map<std::string, geometry_msgs::msg::TransformStamped>
      vr_base_link_dummy_tf_;

  std::map<std::string, tf2::Quaternion> vr_wrist_to_gripper_rot_;

  // Timer for calibration at 10Hz
  rclcpp::TimerBase::SharedPtr calibration_timer_;

  // Timer for VR to gripper TF publishing at 100Hz
  rclcpp::TimerBase::SharedPtr vr_to_gripper_publish_timer_;

  // Joystick subscriber
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;

  // Configurable world frame names
  std::string gripper_world_frame_;
  std::string vr_world_frame_;
};

} // namespace lerobot_vr_controller

#endif // RECEVICE_VR_TF_H_
