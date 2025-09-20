#include "vr_trigger_joint_convert.h"
#include <algorithm>
#include <stdexcept>

namespace vr_controller {

VrTriggerJointConvert::VrTriggerJointConvert() {
  // Constructor - initialize empty joint configurations
}

VrTriggerJointConvert::~VrTriggerJointConvert() {
  // Destructor
}

void VrTriggerJointConvert::AddJointConfig(const TriggerJointConfig &config) {
  if (config.joint_name.empty()) {
    throw std::invalid_argument("Joint name cannot be empty");
  }
  joint_configs_[config.joint_name] = config;
}

double VrTriggerJointConvert::ConvertJointPosition(const std::string &joint_name,
                                                   double trigger_value) const {
  auto it = joint_configs_.find(joint_name);
  if (it == joint_configs_.end()) {
    throw std::runtime_error("Joint '" + joint_name + "' is not configured");
  }

  return CalculateJointPosition(it->second, trigger_value);
}

bool VrTriggerJointConvert::HasJoint(const std::string &joint_name) const {
  return joint_configs_.find(joint_name) != joint_configs_.end();
}

std::vector<std::string> VrTriggerJointConvert::GetConfiguredJoints() const {
  std::vector<std::string> joint_names;
  joint_names.reserve(joint_configs_.size());

  for (const auto &pair : joint_configs_) {
    joint_names.push_back(pair.first);
  }

  return joint_names;
}

void VrTriggerJointConvert::ClearConfigs() { joint_configs_.clear(); }

TriggerJointConfig
VrTriggerJointConvert::GetJointConfig(const std::string &joint_name) const {
  auto it = joint_configs_.find(joint_name);
  if (it == joint_configs_.end()) {
    return TriggerJointConfig(); // Return default config
  }

  return it->second;
}

double
VrTriggerJointConvert::CalculateJointPosition(const TriggerJointConfig &config,
                                              double trigger_value) const {
  // Calculate the delta from the default trigger position
  double trigger_delta = trigger_value - config.trigger_default_position;

  // Apply the scale factor to convert trigger movement to joint movement
  double joint_delta = trigger_delta * config.trigger_to_gripper_scale;

  // Add the delta to the default joint position
  double joint_position = config.default_joint_position + joint_delta;

  return joint_position;
}

} // namespace vr_controller
