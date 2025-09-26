#include "xlerobot_kinematics.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace lerobot_vr_controller {

XLeRobotKinematics::XLeRobotKinematics()
    : timeout_(0.005), position_tolerance_(1e-4), orientation_tolerance_(1e-3),
      num_joints_(5), initialized_(false) {}

bool XLeRobotKinematics::Initialize(const std::string &urdf_param,
                                    const std::string &base_link,
                                    const std::string &tip_link,
                                    double timeout) {
  base_link_ = base_link;
  tip_link_ = tip_link;
  timeout_ = timeout;

  // 初始化关节名称（SO-ARM101标准关节名称，与URDF一致）
  joint_names_ = {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex",
                  "wrist_roll"};

  // 构建关节占位映射
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    joint_placehold_map_[joint_names_[i]] = i;
  }

  // 从URDF内容字符串加载关节限制
  if (!urdf_param.empty() && !LoadURDF(urdf_param)) {
    RCLCPP_WARN(
        rclcpp::get_logger("XLeRobotKinematics"),
        "Failed to parse URDF content string, using default joint limits");
    // 使用默认关节限制作为后备
    SetDefaultJointLimits();
  } else if (urdf_param.empty()) {
    RCLCPP_INFO(rclcpp::get_logger("XLeRobotKinematics"),
                "No URDF content provided, using default joint limits");
    // 没有提供URDF内容，使用默认限制
    SetDefaultJointLimits();
  }

  initialized_ = true;
  return true;
}

bool XLeRobotKinematics::SolveIK(const tf2::Transform &target_transform,
                                 std::vector<double> &solution,
                                 const std::vector<double> &seed_joints) {
  if (!initialized_) {
    RCLCPP_ERROR(rclcpp::get_logger("XLeRobotKinematics"),
                 "Kinematics solver not initialized");
    return false;
  }

  std::lock_guard<std::mutex> lock(solver_mutex_);
  return SolvePolarIK(target_transform, solution);
}

bool XLeRobotKinematics::SolvePolarIK(const tf2::Transform &target_transform,
                                      std::vector<double> &solution) {
  solution.resize(5);

  // 获取目标位置和方向
  tf2::Vector3 target_pos = target_transform.getOrigin();
  double x = target_pos.getX();
  double y = target_pos.getY();
  double z = target_pos.getZ();

  // 步骤1：计算第1关节角度（极坐标旋转角）
  double joint1 = std::atan2(y, x);

  // 步骤2：计算垂直平面内的径向距离
  double r = std::sqrt(x * x + y * y);
  double z_relative = z - kBaseHeight; // 相对于基座的高度

  // 步骤3：使用余弦定理求解第2、3关节角度
  double joint2, joint3;
  if (!SolveVerticalPlane2D(r, z_relative, joint2, joint3)) {
    RCLCPP_WARN(rclcpp::get_logger("XLeRobotKinematics"),
                "Cannot reach target position (r=%f, z=%f)", r, z_relative);
    return false;
  }

  // 步骤4：计算末端朝向角度
  double joint4, joint5;
  if (!CalculateEndEffectorOrientation(target_transform, joint2, joint3, joint4,
                                       joint5)) {
    RCLCPP_WARN(rclcpp::get_logger("XLeRobotKinematics"),
                "Cannot achieve target orientation");
    return false;
  }

  // 组装解
  solution[0] = joint1;
  solution[1] = joint2;
  solution[2] = joint3;
  solution[3] = joint4;
  solution[4] = joint5;

  // 验证关节限制
  return ValidateJoints(solution);
}

bool XLeRobotKinematics::SolveVerticalPlane2D(double r, double z,
                                              double &joint2, double &joint3) {
  // 计算目标点到第2关节的距离
  double target_distance = std::sqrt(r * r + z * z);

  // 检查是否在工作空间内
  double min_reach = std::abs(kLink2Length - kLink3Length);
  double max_reach = kLink2Length + kLink3Length;

  if (target_distance < min_reach || target_distance > max_reach) {
    return false;
  }

  // 使用余弦定理求解第3关节角度
  double cos_joint3 =
      (kLink2Length * kLink2Length + kLink3Length * kLink3Length -
       target_distance * target_distance) /
      (2 * kLink2Length * kLink3Length);

  // 确保cos值在有效范围内
  cos_joint3 = std::max(-1.0, std::min(1.0, cos_joint3));

  // 计算第3关节角度（选择正角度解，确保关节角度>0的约束）
  joint3 = std::acos(cos_joint3);

  // 计算第2关节角度
  double alpha = std::atan2(z, r); // 目标点相对于水平面的角度
  double beta = std::acos((kLink2Length * kLink2Length +
                           target_distance * target_distance -
                           kLink3Length * kLink3Length) /
                          (2 * kLink2Length * target_distance));

  joint2 = alpha - beta;

  return true;
}

bool XLeRobotKinematics::CalculateEndEffectorOrientation(
    const tf2::Transform &target_transform, double joint2, double joint3,
    double &joint4, double &joint5) {

  // 获取目标方向
  tf2::Matrix3x3 target_rotation(target_transform.getRotation());
  double target_roll, target_pitch, target_yaw;
  target_rotation.getRPY(target_roll, target_pitch, target_yaw);

  // 计算前两个关节造成的累积俯仰角
  double accumulated_pitch = joint2 + joint3;

  // 第4关节直接对应俯仰角，需要补偿前面关节的影响
  joint4 = target_pitch - accumulated_pitch;

  // 第5关节直接对应横滚角
  joint5 = target_roll;

  return true;
}

bool XLeRobotKinematics::ValidateJoints(
    const std::vector<double> &joints) const {
  if (joints.size() != num_joints_) {
    return false;
  }

  for (size_t i = 0; i < num_joints_; ++i) {
    if (joints[i] < joint_lower_limits_[i] ||
        joints[i] > joint_upper_limits_[i]) {
      RCLCPP_WARN(rclcpp::get_logger("XLeRobotKinematics"),
                  "Joint %zu out of limits: %f (limits: [%f, %f])", i,
                  joints[i], joint_lower_limits_[i], joint_upper_limits_[i]);
      return false;
    }
  }
  return true;
}

bool XLeRobotKinematics::CheckWorkspace(
    const tf2::Transform &target_transform) const {
  tf2::Vector3 target_pos = target_transform.getOrigin();
  double x = target_pos.getX();
  double y = target_pos.getY();
  double z = target_pos.getZ();

  double r = std::sqrt(x * x + y * y);
  double z_relative = z - kBaseHeight;
  double target_distance = std::sqrt(r * r + z_relative * z_relative);

  double min_reach = std::abs(kLink2Length - kLink3Length);
  double max_reach = kLink2Length + kLink3Length;

  return (target_distance >= min_reach && target_distance <= max_reach);
}

size_t XLeRobotKinematics::GetNumJoints() const { return num_joints_; }

bool XLeRobotKinematics::GetJointLimits(
    std::vector<double> &lower_limits,
    std::vector<double> &upper_limits) const {
  if (!initialized_)
    return false;

  lower_limits = joint_lower_limits_;
  upper_limits = joint_upper_limits_;
  return true;
}

void XLeRobotKinematics::SetTimeout(double timeout) { timeout_ = timeout; }

bool XLeRobotKinematics::IsInitialized() const { return initialized_; }

std::vector<std::string> XLeRobotKinematics::GetJointNames() const {
  return joint_names_;
}

std::map<std::string, int> XLeRobotKinematics::GetJointPlaceholdMap() const {
  return joint_placehold_map_;
}

bool XLeRobotKinematics::AlignJointStateToIk(
    const std::vector<std::string> &joint_names,
    const std::vector<double> &joint_positions,
    std::vector<double> &seed_joints) const {

  if (joint_names.size() != joint_positions.size()) {
    return false;
  }

  seed_joints.resize(num_joints_, 0.0);

  for (size_t i = 0; i < joint_names.size(); ++i) {
    auto it = joint_placehold_map_.find(joint_names[i]);
    if (it != joint_placehold_map_.end()) {
      seed_joints[it->second] = joint_positions[i];
    }
  }

  return true;
}

KDL::Frame
XLeRobotKinematics::TransformToKDLFrame(const tf2::Transform &transform) const {
  KDL::Frame frame;
  frame.p =
      KDL::Vector(transform.getOrigin().getX(), transform.getOrigin().getY(),
                  transform.getOrigin().getZ());

  tf2::Matrix3x3 rot_matrix(transform.getRotation());
  double roll, pitch, yaw;
  rot_matrix.getRPY(roll, pitch, yaw);
  frame.M = KDL::Rotation::RPY(roll, pitch, yaw);

  return frame;
}

bool XLeRobotKinematics::LoadURDF(const std::string &urdf_param) {
  try {
    // 检查URDF内容是否为空
    if (urdf_param.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("XLeRobotKinematics"),
                   "URDF content string is empty");
      return false;
    }

    // 解析URDF模型
    urdf_model_ = std::make_unique<urdf::Model>();
    if (!urdf_model_->initString(urdf_param)) {
      RCLCPP_ERROR(rclcpp::get_logger("XLeRobotKinematics"),
                   "Failed to parse URDF content string");
      return false;
    }

    RCLCPP_INFO(rclcpp::get_logger("XLeRobotKinematics"),
                "Successfully parsed URDF content (length: %zu bytes)",
                urdf_param.length());

    // 解析关节限制
    joint_lower_limits_.resize(num_joints_);
    joint_upper_limits_.resize(num_joints_);

    for (size_t i = 0; i < joint_names_.size(); ++i) {
      const std::string &joint_name = joint_names_[i];
      auto joint = urdf_model_->getJoint(joint_name);

      if (!joint) {
        RCLCPP_ERROR(rclcpp::get_logger("XLeRobotKinematics"),
                     "Joint '%s' not found in URDF content",
                     joint_name.c_str());
        return false;
      }

      if (joint->type == urdf::Joint::REVOLUTE ||
          joint->type == urdf::Joint::CONTINUOUS) {
        if (joint->limits) {
          joint_lower_limits_[i] = joint->limits->lower;
          joint_upper_limits_[i] = joint->limits->upper;
          RCLCPP_INFO(rclcpp::get_logger("XLeRobotKinematics"),
                      "Joint '%s' limits: [%.5f, %.5f]", joint_name.c_str(),
                      joint_lower_limits_[i], joint_upper_limits_[i]);
        } else {
          RCLCPP_WARN(rclcpp::get_logger("XLeRobotKinematics"),
                      "Joint '%s' has no limits defined in URDF",
                      joint_name.c_str());
          return false;
        }
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("XLeRobotKinematics"),
                     "Joint '%s' is not a revolute joint (type: %d)",
                     joint_name.c_str(), joint->type);
        return false;
      }
    }

    RCLCPP_INFO(rclcpp::get_logger("XLeRobotKinematics"),
                "Successfully loaded joint limits from URDF content");
    return true;

  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("XLeRobotKinematics"),
                 "Exception while parsing URDF content: %s", e.what());
    return false;
  }
}

void XLeRobotKinematics::SetDefaultJointLimits() {
  // 设置默认关节限制（SO-ARM101标准限制，单位：弧度，与URDF一致）
  // shoulder_pan: -1.91986 to 1.91986
  // shoulder_lift: -1.74533 to 1.74533
  // elbow_flex: -1.69 to 1.69
  // wrist_flex: -1.65806 to 1.65806
  // wrist_roll: -2.74385 to 2.84121
  joint_lower_limits_ = {-1.91986, -1.74533, -1.69, -1.65806, -2.74385};
  joint_upper_limits_ = {1.91986, 1.74533, 1.69, 1.65806, 2.84121};

  RCLCPP_INFO(rclcpp::get_logger("XLeRobotKinematics"),
              "Using default joint limits");
}

} // namespace lerobot_vr_controller
