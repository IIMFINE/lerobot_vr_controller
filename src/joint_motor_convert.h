#ifndef JOINT_MOTOR_CONVERT_H_
#define JOINT_MOTOR_CONVERT_H_

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace lerobot_vr_controller {

struct JointMotorMapping {
  double motor_pos;
  double joint_pos;
};

struct JointMotorScale {
  double scale;
};

struct MotorCalibration {
  int id;
  int drive_mode;
  int homing_offset;
  int range_min;
  int range_max;
};

struct JointRange {
  double min_angle; // radians
  double max_angle; // radians
};

class JointMotorConvert {
public:
  explicit JointMotorConvert(
      const std::string &config_file_path,
      const std::string &motor_calibration_file_path = "");
  virtual ~JointMotorConvert() = default;

  // Convert joint position (radians) to motor position
  virtual double JointToMotorPosition(const std::string &joint_name,
                                      double joint_position) const;

  // Convert motor position to joint position (radians)
  virtual double MotorToJointPosition(const std::string &joint_name,
                                      double motor_position) const;

  // Get all supported joint names
  virtual std::vector<std::string> GetJointNames() const;

  // Check if joint name is valid
  virtual bool IsValidJoint(const std::string &joint_name) const;

  // Get motor range limits for a joint
  virtual bool GetMotorRange(const std::string &joint_name, int &range_min,
                             int &range_max) const;

  // Get joint angle range limits for a joint
  virtual bool GetJointRange(const std::string &joint_name, double &min_angle,
                             double &max_angle) const;

  // Get motor calibration data for a joint
  virtual bool GetMotorCalibration(const std::string &joint_name,
                                   MotorCalibration &calibration) const;

  // Load motor calibration data from JSON file
  virtual bool LoadMotorCalibration(const std::string &calibration_file_path);

protected:
  // Load configuration from YAML file
  virtual bool LoadConfiguration(const std::string &config_file_path);

  std::map<std::string, JointMotorMapping> joint_motor_mappings_;
  std::map<std::string, JointMotorScale> joint_motor_scales_;
  std::map<std::string, MotorCalibration> motor_calibrations_;
  std::map<std::string, JointRange> joint_ranges_;
};

} // namespace lerobot_vr_controller

#endif // JOINT_MOTOR_CONVERT_H_
