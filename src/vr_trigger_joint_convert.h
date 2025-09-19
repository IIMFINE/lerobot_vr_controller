#ifndef VR_TRIGGER_JOINT_CONVERT_H
#define VR_TRIGGER_JOINT_CONVERT_H

#include <map>
#include <string>
#include <vector>

namespace vr_controller {

/**
 * @brief Configuration structure for VR trigger to joint conversion
 */
struct TriggerJointConfig {
  std::string joint_name;          // Name of the joint to control
  double default_joint_position;   // Default position of the joint
  double trigger_default_position; // Default trigger position (usually 0.0)
  double
      trigger_to_gripper_scale; // Scale factor from trigger to joint position

  TriggerJointConfig()
      : joint_name(""), default_joint_position(0.0),
        trigger_default_position(0.0), trigger_to_gripper_scale(0.0068) {}

  TriggerJointConfig(const std::string &name, double default_pos,
                     double trigger_default, double scale)
      : joint_name(name), default_joint_position(default_pos),
        trigger_default_position(trigger_default),
        trigger_to_gripper_scale(scale) {}
};

/**
 * @brief Class for converting VR trigger input to joint positions
 */
class VrTriggerJointConvert {
public:
  /**
   * @brief Constructor
   */
  VrTriggerJointConvert();

  /**
   * @brief Destructor
   */
  ~VrTriggerJointConvert();

  /**
   * @brief Add or update a joint configuration
   * @param config The trigger joint configuration
   */
  void AddJointConfig(const TriggerJointConfig &config);

  /**
   * @brief Get joint position for a given trigger value
   * @param joint_name Name of the joint
   * @param trigger_value Current trigger value (typically 0.0 to 1.0)
   * @return Calculated joint position
   */
  double GetJointPosition(const std::string &joint_name,
                          double trigger_value) const;

  /**
   * @brief Check if a joint is configured
   * @param joint_name Name of the joint
   * @return true if joint is configured, false otherwise
   */
  bool HasJoint(const std::string &joint_name) const;

  /**
   * @brief Get all configured joint names
   * @return Vector of joint names
   */
  std::vector<std::string> GetConfiguredJoints() const;

  /**
   * @brief Clear all joint configurations
   */
  void ClearConfigs();

  /**
   * @brief Get the configuration for a specific joint
   * @param joint_name Name of the joint
   * @return The configuration struct, or default config if not found
   */
  TriggerJointConfig GetJointConfig(const std::string &joint_name) const;

private:
  std::map<std::string, TriggerJointConfig> joint_configs_;

  /**
   * @brief Calculate joint position based on trigger value and configuration
   * @param config The joint configuration
   * @param trigger_value Current trigger value
   * @return Calculated joint position
   */
  double CalculateJointPosition(const TriggerJointConfig &config,
                                double trigger_value) const;
};

} // namespace vr_controller

#endif // VR_TRIGGER_JOINT_CONVERT_H
