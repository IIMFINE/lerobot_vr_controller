#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>

#include "ik.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"

int main(int argc, char **argv) {
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Create node
  auto node = std::make_shared<rclcpp::Node>("ik_test_node");

  RCLCPP_INFO(node->get_logger(), "Starting IK Test Node");

  // Create SoArm101Kinematics instance
  auto kinematics =
      std::make_unique<lerobot_vr_controller::SoArm101Kinematics>();

  // Get URDF file path from parameter server
  std::string urdf_file;
  node->declare_parameter("urdf_file", "");

  // Wait for urdf_file parameter to be available
  auto start_time = node->get_clock()->now();
  auto timeout = rclcpp::Duration::from_seconds(10.0);

  while (rclcpp::ok() && (node->get_clock()->now() - start_time) < timeout) {
    if (node->get_parameter("urdf_file", urdf_file) && !urdf_file.empty()) {
      break;
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for urdf_file parameter...");
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    rclcpp::spin_some(node);
  }

  if (urdf_file.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get urdf_file parameter");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(),
              "Successfully received urdf_file parameter: %s",
              urdf_file.c_str());

  // Read URDF file content
  std::ifstream urdf_stream(urdf_file);
  if (!urdf_stream.is_open()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to open URDF file: %s",
                 urdf_file.c_str());
    rclcpp::shutdown();
    return 1;
  }

  std::string robot_description((std::istreambuf_iterator<char>(urdf_stream)),
                                std::istreambuf_iterator<char>());
  urdf_stream.close();

  if (robot_description.empty()) {
    RCLCPP_ERROR(node->get_logger(), "URDF file is empty: %s",
                 urdf_file.c_str());
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Successfully loaded URDF content from file");

  // Initialize the kinematics solver with URDF string
  if (!kinematics->Initialize(robot_description, "base_link", "gripper_link")) {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize kinematics solver");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Kinematics solver initialized successfully");

  // Get joint information
  size_t num_joints = kinematics->GetNumJoints();
  RCLCPP_INFO(node->get_logger(), "Number of joints: %zu", num_joints);

  std::vector<std::string> joint_names = kinematics->GetJointNames();
  RCLCPP_INFO(node->get_logger(), "Joint names:");
  for (size_t i = 0; i < joint_names.size(); ++i) {
    RCLCPP_INFO(node->get_logger(), "  [%zu]: %s", i, joint_names[i].c_str());
  }

  // Get joint limits
  std::vector<double> lower_limits, upper_limits;
  if (kinematics->GetJointLimits(lower_limits, upper_limits)) {
    RCLCPP_INFO(node->get_logger(), "Joint limits:");
    for (size_t i = 0; i < num_joints; ++i) {
      RCLCPP_INFO(node->get_logger(), "  Joint %zu: [%.3f, %.3f] rad", i,
                  lower_limits[i], upper_limits[i]);
    }
  }

  // Initialize seed joints to a reasonable configuration instead of all zeros
  std::vector<double> seed_joints(num_joints);
  if (num_joints >= 5) {
    // 使用一个更合理的初始姿态（假设关节顺序为：shoulder_pan, shoulder_lift,
    // elbow_flex, wrist_flex, wrist_roll）
    seed_joints[0] = 0.0; // shoulder_pan: 正前方
    seed_joints[1] = 0.3; // shoulder_lift: 稍微抬起
    seed_joints[2] = 0.0; // elbow_flex: 伸直
    seed_joints[3] = 0.0; // wrist_flex: 中性
    seed_joints[4] = 0.0; // wrist_roll: 中性
  } else {
    // 如果关节数不是5，则使用全零
    std::fill(seed_joints.begin(), seed_joints.end(), 0.0);
  }

  RCLCPP_INFO(node->get_logger(), "Using initial joint configuration:");
  for (size_t i = 0; i < num_joints; ++i) {
    RCLCPP_INFO(node->get_logger(), "  Joint %zu: %.3f rad", i, seed_joints[i]);
  }

  // Test cases: Different target positions and orientations (更保守的目标位置)
  std::vector<std::tuple<tf2::Vector3, tf2::Quaternion, std::string>>
      test_cases = {
          // Test case 1: 更近的前方位置
          {tf2::Vector3(0.15, 0.0, 0.15), tf2::Quaternion(0, 0, 0, 1),
           "Close forward position"},

          // Test case 2: 很小的侧向位置
          {tf2::Vector3(0.12, 0.05, 0.12), tf2::Quaternion(0, 0, 0, 1),
           "Small side position"},

          // Test case 3: 稍高一点的位置
          {tf2::Vector3(0.1, 0.0, 0.2), tf2::Quaternion(0, 0, 0, 1),
           "Slightly higher position"},

          // Test case 4: 基础位置，无旋转
          {tf2::Vector3(0.08, 0.0, 0.1), tf2::Quaternion(0, 0, 0, 1),
           "Very close position"},
      };

  // Run test cases
  for (size_t test_idx = 0; test_idx < test_cases.size(); ++test_idx) {
    auto [position, orientation, description] = test_cases[test_idx];

    RCLCPP_INFO(node->get_logger(), "\n=== Test Case %zu: %s ===", test_idx + 1,
                description.c_str());

    // Create target transform
    tf2::Transform target_transform;
    target_transform.setOrigin(position);
    target_transform.setRotation(orientation);

    RCLCPP_INFO(node->get_logger(), "Target position: [%.3f, %.3f, %.3f]",
                position.x(), position.y(), position.z());
    RCLCPP_INFO(node->get_logger(),
                "Target orientation: [%.3f, %.3f, %.3f, %.3f]", orientation.x(),
                orientation.y(), orientation.z(), orientation.w());

    // Solve IK
    std::vector<double> solution;
    bool success = kinematics->SolveIK(target_transform, solution, seed_joints);

    if (success) {
      RCLCPP_INFO(node->get_logger(), "IK Solution found:");
      for (size_t i = 0; i < solution.size(); ++i) {
        RCLCPP_INFO(node->get_logger(), "  %s: %.4f rad (%.2f deg)",
                    joint_names[i].c_str(), solution[i],
                    solution[i] * 180.0 / M_PI);
      }

      // Use this solution as seed for next iteration
      seed_joints = solution;
    } else {
      RCLCPP_WARN(node->get_logger(), "IK Solution NOT found for this target");
    }

    // Add small delay between tests
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  // Test workspace checking
  RCLCPP_INFO(node->get_logger(), "\n=== Workspace Check Tests ===");

  std::vector<std::tuple<tf2::Vector3, std::string>> workspace_tests = {
      {tf2::Vector3(0.1, 0.0, 0.1), "Close position"},
      {tf2::Vector3(0.5, 0.0, 0.3), "Medium position"},
      {tf2::Vector3(1.0, 0.0, 0.5), "Far position (likely out of reach)"},
      {tf2::Vector3(0.0, 0.0, -0.1), "Below base (invalid)"},
  };

  for (const auto &[pos, desc] : workspace_tests) {
    tf2::Transform test_transform;
    test_transform.setOrigin(pos);
    test_transform.setRotation(tf2::Quaternion(0, 0, 0, 1));

    bool in_workspace = kinematics->CheckWorkspace(test_transform);
    RCLCPP_INFO(node->get_logger(), "%s [%.2f, %.2f, %.2f]: %s", desc.c_str(),
                pos.x(), pos.y(), pos.z(),
                in_workspace ? "IN workspace" : "OUT of workspace");
  }

  RCLCPP_INFO(node->get_logger(), "\n=== IK Test Completed ===");

  // Keep node alive for a bit to see results
  std::this_thread::sleep_for(std::chrono::seconds(2));

  rclcpp::shutdown();
  return 0;
}
