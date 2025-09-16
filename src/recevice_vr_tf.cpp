#include "recevice_vr_tf.h"

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace lerobot_vr_controller {

VrTfReceiver::VrTfReceiver(std::shared_ptr<rclcpp::Node> node) : node_(node) {
  // Initialize TF broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

  // Initialize TF buffer and listener
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(node_->get_logger(), "VrTfReceiver initialized");
}

bool VrTfReceiver::InitializeYamlConfig(const std::string &yaml_file_path) {
  yaml_config_path_ = yaml_file_path;

  if (!LoadYamlConfig(yaml_config_path_)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Failed to load YAML configuration from: %s",
                 yaml_config_path_.c_str());
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "YAML configuration loaded successfully");
  return true;
}

void VrTfReceiver::Start() {
  // Create timer to execute CalibrateVr2GripperTf at 10Hz (100ms interval)
  calibration_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&VrTfReceiver::CalibrateVr2GripperTf, this));

  // Initialize VR to gripper TF publishing timer at 100Hz
  vr_to_gripper_publish_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&VrTfReceiver::Vr2GripperTfPublish, this));
}

void VrTfReceiver::PublishTransform(const std::string &parent_frame,
                                    const std::string &child_frame,
                                    const geometry_msgs::msg::Pose &pose,
                                    const rclcpp::Time &timestamp) {

  geometry_msgs::msg::TransformStamped transform_stamped;

  transform_stamped.header.stamp = timestamp;
  transform_stamped.header.frame_id = parent_frame;
  transform_stamped.child_frame_id = child_frame;

  transform_stamped.transform.translation.x = pose.position.x;
  transform_stamped.transform.translation.y = pose.position.y;
  transform_stamped.transform.translation.z = pose.position.z;

  transform_stamped.transform.rotation = pose.orientation;

  tf_broadcaster_->sendTransform(transform_stamped);

  RCLCPP_DEBUG(node_->get_logger(), "Published transform from %s to %s",
               parent_frame.c_str(), child_frame.c_str());
}

bool VrTfReceiver::LoadYamlConfig(const std::string &yaml_file_path) {
  try {
    YAML::Node config = YAML::LoadFile(yaml_file_path);

    if (!config["vr_to_arm_tf"]) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Missing 'vr_to_arm_tf' section in YAML file");
      return false;
    }

    auto tf_config = config["vr_to_arm_tf"];

    // Load world frame configurations
    if (config["gripper_world_frame"]) {
      gripper_world_frame_ = config["gripper_world_frame"].as<std::string>();
      RCLCPP_INFO(node_->get_logger(), "gripper_world_frame: %s",
                  gripper_world_frame_.c_str());
    } else {
      gripper_world_frame_ = "world"; // default fallback
    }

    if (config["vr_world_frame"]) {
      vr_world_frame_ = config["vr_world_frame"].as<std::string>();
      RCLCPP_INFO(node_->get_logger(), "vr_world_frame: %s",
                  vr_world_frame_.c_str());
    } else {
      vr_world_frame_ = "world"; // default fallback
    }

    RCLCPP_INFO(node_->get_logger(), "Using gripper world frame: %s",
                gripper_world_frame_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Using VR world frame: %s",
                vr_world_frame_.c_str());

    // Iterate through all key-value pairs in vr_to_arm_tf
    for (auto it = tf_config.begin(); it != tf_config.end(); ++it) {
      std::string gripper_link = it->first.as<std::string>();
      std::string vr_link = it->second.as<std::string>();

      gripper_link_to_vr_map_[gripper_link] = vr_link;

      RCLCPP_INFO(node_->get_logger(), "Loaded mapping: %s -> %s",
                  vr_link.c_str(), gripper_link.c_str());
    }

    return true;

  } catch (const YAML::Exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "YAML parsing error: %s", e.what());
    return false;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node_->get_logger(), "Error loading YAML config: %s",
                 e.what());
    return false;
  }
}

void VrTfReceiver::CalibrateVr2GripperTf() {
  if (!tf_buffer_ || !tf_listener_) {
    RCLCPP_ERROR(node_->get_logger(), "TF buffer or listener not initialized");
    return;
  }

  if (calibrated_flag_) {
    return;
  }

  // Clear previous transformations
  vr_to_gripper_tf_.clear();

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

      // Compute vr to gripper transform
      tf2::Transform tf_vr_to_gripper =
          tf_world_to_vr.inverse() * tf_world_to_gripper;

      // Convert back to TransformStamped message
      geometry_msgs::msg::TransformStamped vr_to_gripper_msg;
      vr_to_gripper_msg.header.frame_id = vr_frame;
      vr_to_gripper_msg.child_frame_id = gripper_link + "_calibrated";
      vr_to_gripper_msg.header.stamp = node_->now();
      vr_to_gripper_msg.transform = tf2::toMsg(tf_vr_to_gripper);

      // Store the transformation
      vr_to_gripper_tf_[gripper_link] = vr_to_gripper_msg;

      calibrated_flag_ = true;

      // RCLCPP_INFO(node_->get_logger(),
      //             "Successfully computed and published transform from %s to
      //             %s", vr_frame.c_str(), gripper_link.c_str());

    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Failed to get transform for gripper_link: %s, vr_frame: "
                   "%s. Error: %s",
                   gripper_link.c_str(), vr_frame.c_str(), ex.what());
    }
  }

  if (!vr_to_gripper_tf_.empty()) {
    RCLCPP_INFO(node_->get_logger(),
                "Successfully computed %zu VR to gripper transformations",
                vr_to_gripper_tf_.size());
  }
}

void VrTfReceiver::Vr2GripperTfPublish() {
  if (!calibrated_flag_) {
    return;
  }

  for (const auto &[gripper_link, vr_frame] : gripper_link_to_vr_map_) {
    // Get current VR transform
    try {
      geometry_msgs::msg::TransformStamped vr_transform;
      vr_transform = tf_buffer_->lookupTransform(vr_world_frame_, vr_frame,
                                                 tf2::TimePointZero);

      // Apply calibration transformation
      auto it = vr_to_gripper_tf_.find(gripper_link);
      if (it != vr_to_gripper_tf_.end()) {
        // Create child frame name with "gripper_cal" suffix
        std::string child_frame = gripper_link + "_gripper_cal";

        // Apply the stored calibration transformation using tf2
        tf2::Transform tf_vr_current;
        tf2::Transform tf_vr_to_gripper;

        tf2::fromMsg(vr_transform.transform, tf_vr_current);
        tf2::fromMsg(it->second.transform, tf_vr_to_gripper);

        // Compute the calibrated gripper transform
        tf2::Transform tf_calibrated_gripper = tf_vr_current * tf_vr_to_gripper;

        // Convert back to geometry_msgs and publish
        geometry_msgs::msg::TransformStamped calibrated_transform;
        calibrated_transform.header.stamp = node_->now();
        calibrated_transform.header.frame_id = gripper_world_frame_;
        calibrated_transform.child_frame_id = child_frame;
        calibrated_transform.transform = tf2::toMsg(tf_calibrated_gripper);

        tf_broadcaster_->sendTransform(calibrated_transform);
      }
    } catch (tf2::TransformException &ex) {
      // Silently continue if transform not available
    }
  }
}

} // namespace lerobot_vr_controller
