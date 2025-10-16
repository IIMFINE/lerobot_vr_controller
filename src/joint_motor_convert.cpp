#include "joint_motor_convert.h"

#include "log.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace lerobot_vr_controller {

JointMotorConvert::JointMotorConvert(
    const std::string &config_file_path,
    const std::string &motor_calibration_file_path) {
  if (!LoadConfiguration(config_file_path)) {
    LE_LOG_ERROR << "Failed to load configuration from: " << config_file_path
                 << std::endl;
  }

  // Load motor calibration file
  if (!motor_calibration_file_path.empty()) {
    LoadMotorCalibration(motor_calibration_file_path);
  }
}

double JointMotorConvert::JointToMotorPosition(const std::string &joint_name,
                                               double joint_position) const {
  auto mapping_it = joint_motor_mappings_.find(joint_name);
  auto scale_it = joint_motor_scales_.find(joint_name);

  if (mapping_it == joint_motor_mappings_.end() ||
      scale_it == joint_motor_scales_.end()) {
    LE_LOG_ERROR << "Joint " << joint_name << " not found in configuration"
                 << std::endl;
    return 0.0;
  }

  const auto &mapping = mapping_it->second;
  const auto &scale = scale_it->second;

  // Formula: motor_pos = mapping.motor_pos + (joint_pos - mapping.joint_pos) *
  // scale.scale
  double motor_position =
      mapping.motor_pos + (joint_position - mapping.joint_pos) * scale.scale;

  return motor_position;
}

double JointMotorConvert::MotorToJointPosition(const std::string &joint_name,
                                               double motor_position) const {
  auto mapping_it = joint_motor_mappings_.find(joint_name);
  auto scale_it = joint_motor_scales_.find(joint_name);

  if (mapping_it == joint_motor_mappings_.end() ||
      scale_it == joint_motor_scales_.end()) {
    LE_LOG_ERROR_T(5s) << "Joint " << joint_name << " not found in configuration"
                 << std::endl;
    return 0.0;
  }

  const auto &mapping = mapping_it->second;
  const auto &scale = scale_it->second;

  // Formula: joint_pos = mapping.joint_pos + (motor_pos - mapping.motor_pos) /
  // scale.scale
  double joint_position =
      mapping.joint_pos + (motor_position - mapping.motor_pos) / scale.scale;

  return joint_position;
}

std::vector<std::string> JointMotorConvert::GetJointNames() const {
  std::vector<std::string> joint_names;
  joint_names.reserve(joint_motor_mappings_.size());

  for (const auto &pair : joint_motor_mappings_) {
    joint_names.push_back(pair.first);
  }

  return joint_names;
}

bool JointMotorConvert::IsValidJoint(const std::string &joint_name) const {
  return joint_motor_mappings_.find(joint_name) != joint_motor_mappings_.end();
}

bool JointMotorConvert::GetMotorRange(const std::string &joint_name,
                                      int &range_min, int &range_max) const {
  auto it = motor_calibrations_.find(joint_name);
  if (it == motor_calibrations_.end()) {
    LE_LOG_ERROR << "Motor calibration for joint " << joint_name << " not found"
                 << std::endl;
    return false;
  }

  range_min = it->second.range_min;
  range_max = it->second.range_max;
  return true;
}

bool JointMotorConvert::GetJointRange(const std::string &joint_name,
                                      double &min_angle,
                                      double &max_angle) const {
  auto it = joint_ranges_.find(joint_name);
  if (it == joint_ranges_.end()) {
    LE_LOG_ERROR << "Joint range for joint " << joint_name << " not found"
                 << std::endl;
    return false;
  }

  min_angle = it->second.min_angle;
  max_angle = it->second.max_angle;
  return true;
}

bool JointMotorConvert::GetMotorCalibration(
    const std::string &joint_name, MotorCalibration &calibration) const {
  auto it = motor_calibrations_.find(joint_name);
  if (it == motor_calibrations_.end()) {
    LE_LOG_ERROR << "Motor calibration for joint " << joint_name << " not found"
                 << std::endl;
    return false;
  }

  calibration = it->second;
  return true;
}

bool JointMotorConvert::LoadMotorCalibration(
    const std::string &calibration_file_path) {
  try {
    std::ifstream file(calibration_file_path);
    if (!file.is_open()) {
      LE_LOG_ERROR << "Could not open motor calibration file: "
                   << calibration_file_path << std::endl;
      return false;
    }

    nlohmann::json j;
    file >> j;

    // Clear existing data
    motor_calibrations_.clear();
    joint_ranges_.clear();

    // Parse each joint's calibration data
    for (auto &[joint_name, joint_data] : j.items()) {
      MotorCalibration calibration;
      calibration.id = joint_data["id"];
      calibration.drive_mode = joint_data["drive_mode"];
      calibration.homing_offset = joint_data["homing_offset"];
      calibration.range_min = joint_data["range_min"];
      calibration.range_max = joint_data["range_max"];

      motor_calibrations_[joint_name] = calibration;

      // Calculate corresponding joint angle ranges
      JointRange joint_range;
      joint_range.min_angle =
          MotorToJointPosition(joint_name, calibration.range_min);
      joint_range.max_angle =
          MotorToJointPosition(joint_name, calibration.range_max);

      joint_ranges_[joint_name] = joint_range;

      LE_LOG_INFO << "Loaded calibration for " << joint_name << ":"
                  << std::endl;
      LE_LOG_INFO << "  Motor range: [" << calibration.range_min << ", "
                  << calibration.range_max << "]" << std::endl;
      LE_LOG_INFO << "  Joint range: [" << joint_range.min_angle << ", "
                  << joint_range.max_angle << "] rad" << std::endl;
      LE_LOG_INFO << "  Joint range: [" << joint_range.min_angle * 180.0 / M_PI
                  << ", " << joint_range.max_angle * 180.0 / M_PI << "] deg"
                  << std::endl;
    }

    return true;
  } catch (const std::exception &e) {
    LE_LOG_ERROR << "Error loading motor calibration: " << e.what()
                 << std::endl;
    return false;
  }
}

bool JointMotorConvert::LoadConfiguration(const std::string &config_file_path) {
  try {
    YAML::Node config = YAML::LoadFile(config_file_path);

    // Clear existing data
    joint_motor_mappings_.clear();
    joint_motor_scales_.clear();

    // Load joint motor position mappings
    if (config["joint_motor_pos_mapping"]) {
      for (const auto &joint : config["joint_motor_pos_mapping"]) {
        std::string joint_name = joint.first.as<std::string>();
        JointMotorMapping mapping;
        mapping.motor_pos = joint.second["motor_pos"].as<double>();
        mapping.joint_pos = joint.second["joint_pos"].as<double>();
        joint_motor_mappings_[joint_name] = mapping;
      }
    }

    // Load joint to motor scales
    if (config["joint_to_motor_scale"]) {
      for (const auto &joint : config["joint_to_motor_scale"]) {
        std::string joint_name = joint.first.as<std::string>();
        JointMotorScale scale;
        scale.scale = joint.second.as<double>();
        joint_motor_scales_[joint_name] = scale;
      }
    }

    LE_LOG_INFO << "Successfully loaded configuration from: "
                << config_file_path << std::endl;
    return true;
  } catch (const std::exception &e) {
    LE_LOG_ERROR << "Error loading configuration: " << e.what() << std::endl;
    return false;
  }
}

} // namespace lerobot_vr_controller
