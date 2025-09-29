#include "xlerobot_kinematics.h"

#include <algorithm>
#include <functional>
#include <iomanip>
#include <limits>

#include "common_math.h"
#include "log.h"

namespace lerobot_vr_controller {

XLeRobotKinematics::XLeRobotKinematics()
    : base_to_shoulder_pan_height_(0.0624),
      base_to_shoulder_lift_height_(0.0624), shoulder_pan_offset_(0.0),
      upper_arm_length_(0.11257), lower_arm_length_(0.1349),
      wrist_flex_to_roll_offset_(0.0611), wrist_roll_to_gripper_offset_(0.0181),
      min_arm_reach_(0.0), max_arm_reach_(0.0), urdf_model_(nullptr),
      position_tolerance_(1e-4), orientation_tolerance_(1e-3), num_joints_(5),
      initialized_(false) {
  // 初始化关节RPY偏移量（默认值，将从URDF更新）
  joint_origin_rpy_offsets_.resize(5, 0.0);
  tip_link_initial_transform_.setIdentity();
  ee_link_initial_transform_.setIdentity();
}

bool XLeRobotKinematics::Initialize(
    const std::string &urdf_param,
    [[maybe_unused]] const std::string &base_link,
    [[maybe_unused]] const std::string &tip_link,
    [[maybe_unused]] double timeout) {

  // 初始化关节名称（SO-ARM101标准关节名称，与URDF一致）
  joint_names_ = {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex",
                  "wrist_roll"};

  // 构建关节占位映射
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    joint_placehold_map_[joint_names_[i]] = i;
  }

  // 先检查URDF内容是否为空
  if (urdf_param.empty()) {
    LE_LOG_INFO << "No URDF content provided, using default joint limits"
                << std::endl;
    SetDefaultJointLimits();
    initialized_ = true;
    return true;
  }

  // 尝试加载URDF
  if (!LoadURDF(urdf_param)) {
    LE_LOG_ERROR
        << "Failed to parse URDF content string, using default joint limits"
        << std::endl;
    SetDefaultJointLimits();
    initialized_ = true;
    return true;
  }

  // URDF成功加载，解析机械臂结构参数
  if (!ParseKinematicsParameters()) {
    LE_LOG_ERROR << "Failed to parse kinematics parameters from URDF, using "
                    "default values"
                 << std::endl;
  }

  // 同时解析 tip link 和 end effector 的初始朝向（它们不是同一个东西）
  if (!ParseTipLinkInitialOrientation(base_link, tip_link)) {
    LE_LOG_ERROR << "Failed to parse tip link initial orientation for '"
                 << tip_link << "'" << std::endl;
  }

  if (!end_effector_frame_.empty()) {
    if (!ParseEeFrameInitialOrientation(base_link)) {
      LE_LOG_ERROR << "Failed to parse end effector initial orientation for '"
                   << end_effector_frame_ << "'" << std::endl;
    }
  }

  initialized_ = true;

  // 打印所有机械臂长度参数
  PrintArmLengthParameters();
  
  return true;
}

bool XLeRobotKinematics::SolveIK(
    const tf2::Transform &target_transform, std::vector<double> &solution,
    [[maybe_unused]] const std::vector<double> &seed_joints) {
  if (!initialized_) {
    LE_LOG_ERROR << "Kinematics solver not initialized" << std::endl;
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

  // 步骤1：计算第0关节角度（极坐标旋转角）
  // 考虑URDF中shoulder_pan的origin偏移
  // 与关节的正方向定义一致，添加负号
  double shoulder_pan_radian = -std::atan2(y, x);

  // 步骤2：计算垂直平面内的径向距离
  double r = std::sqrt(x * x + y * y);
  double z_relative = z - base_to_shoulder_lift_height_; // 相对于基座的高度

  // 步骤3：使用余弦定理求解第1、2关节角度，考虑URDF偏移
  double shoulder_lift_radian, elbow_flex_radian;
  if (!SolveVerticalPlane2D(r, z_relative, shoulder_lift_radian,
                            elbow_flex_radian)) {
    LE_LOG_ERROR << "Cannot reach target position (r=" << r
                 << ", z=" << z_relative << ")" << std::endl;
    return false;
  }

  // 步骤4：计算末端朝向角度，考虑URDF偏移
  auto [wrist_flex_radian, wrist_roll_radian] = CalculateEndEffectorOrientation(
      target_transform, shoulder_lift_radian, elbow_flex_radian);

  // 步骤5：直接使用计算得到的关节位置（已经考虑了URDF偏移）
  // 这样确保关节位置为0时，机械臂呈现URDF中定义的几何配置
  solution[0] = shoulder_pan_radian + joint_origin_rpy_offsets_[0];
  solution[1] = shoulder_lift_radian - joint_origin_rpy_offsets_[1];
  solution[2] = joint_origin_rpy_offsets_[2] - elbow_flex_radian;
  solution[3] = wrist_flex_radian - joint_origin_rpy_offsets_[3];
  solution[4] = wrist_roll_radian - joint_origin_rpy_offsets_[4];

  // 仅使用URDF的joint_origin_rpy_offsets_进行求解，不再额外添加输出偏移

  // 标准化角度到 [-π, π] 范围内
  for (int i = 0; i < 5; ++i) {
    while (solution[i] > M_PI)
      solution[i] -= 2.0 * M_PI;
    while (solution[i] < -M_PI)
      solution[i] += 2.0 * M_PI;
  }

  // 验证关节限制
  return ValidateJoints(solution);
}

bool XLeRobotKinematics::SolveVerticalPlane2D(double r, double z,
                                              double &shoulder_lift_radian,
                                              double &elbow_flex_radian) {
  // 计算目标点到第1关节的距离
  double target_distance = std::sqrt(r * r + z * z);

  // 检查是否在工作空间内
  if (target_distance < min_arm_reach_ || target_distance > max_arm_reach_) {
    LE_LOG_ERROR << "Target distance " << target_distance
                 << " out of reach (min: " << min_arm_reach_
                 << ", max: " << max_arm_reach_ << ")" << std::endl;
    return false;
  }

  // 使用余弦定理求解肘部弯曲角度
  double cos_elbow_flex = (upper_arm_length_ * upper_arm_length_ +
                           lower_arm_length_ * lower_arm_length_ -
                           target_distance * target_distance) /
                          (2 * upper_arm_length_ * lower_arm_length_);

  // 计算肘部弯曲角度（选择正角度解，确保关节角度>0的约束）
  elbow_flex_radian = std::acos(cos_elbow_flex);

  // 计算肩部抬升角度
  double alpha = std::atan2(r, z); // 目标点相对于水平面的角度
  double beta = std::acos((upper_arm_length_ * upper_arm_length_ +
                           target_distance * target_distance -
                           lower_arm_length_ * lower_arm_length_) /
                          (2 * upper_arm_length_ * target_distance));

  // 根据肘部弯曲角度决定肩部抬升角度的计算方式
  // 保证 shoulder_lift 向前是正数，向后是负数
  shoulder_lift_radian = alpha - beta;

  return true;
}

std::pair<double, double> XLeRobotKinematics::CalculateEndEffectorOrientation(
    const tf2::Transform &target_transform, double shoulder_lift_radian,
    double elbow_flex_radian) {

  // 获取目标方向
  const double target_pitch =
      GetIntersectionAngle(target_transform, Plane::kXY, Axis::kZ);

  // 计算前两个关节造成的累积俯仰角，
  // 当 lower_arm_link 的朝向向下时，accumulated_pitch是负数，朝上则是正数。
  double accumulated_pitch = elbow_flex_radian - shoulder_lift_radian - M_PI_2;

  // 第3关节直接对应俯仰角，需要补偿前面关节的影响
  // 第3关节，wrist_flex 向下是正关节角度，向上是负关节角度
  //而 lower_arm_link 向下是负数，target_pitch
  //向下也是负数，所以要取反来获取正确的 wrist_flex 关节角度
  double wrist_flex_radian = accumulated_pitch - target_pitch;

  // 第4关节计算
  double wrist_roll_radian = GetYaw(target_transform);

  // 限制wrist_roll_radian在关节限制范围内
  constexpr size_t kWristRollIndex = 4;
  if (kWristRollIndex < joint_lower_limits_.size() &&
      kWristRollIndex < joint_upper_limits_.size()) {
    wrist_roll_radian =
        std::clamp(wrist_roll_radian, joint_lower_limits_[kWristRollIndex],
                   joint_upper_limits_[kWristRollIndex]);
  }

  return std::make_pair(wrist_flex_radian, wrist_roll_radian);
}

bool XLeRobotKinematics::ValidateJoints(
    const std::vector<double> &joints) const {
  if (joints.size() != num_joints_) {
    return false;
  }

  for (size_t i = 0; i < num_joints_; ++i) {
    if (joints[i] < joint_lower_limits_[i] ||
        joints[i] > joint_upper_limits_[i]) {
      LE_LOG_ERROR << "Joint " << i << " out of limits: " << joints[i]
                   << " (limits: [" << joint_lower_limits_[i] << ", "
                   << joint_upper_limits_[i] << "])" << std::endl;
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
  double z_relative = z - base_to_shoulder_lift_height_;
  double target_distance = std::sqrt(r * r + z_relative * z_relative);

  return (target_distance >= min_arm_reach_ &&
          target_distance <= max_arm_reach_);
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
      LE_LOG_ERROR << "URDF content string is empty" << std::endl;
      return false;
    }

    // 解析URDF模型
    urdf_model_ = std::make_unique<urdf::Model>();
    if (!urdf_model_->initString(urdf_param)) {
      LE_LOG_ERROR << "Failed to parse URDF content string" << std::endl;
      return false;
    }

    LE_LOG_INFO << "Successfully parsed URDF content (length: "
                << urdf_param.length() << " bytes)" << std::endl;

    // 解析关节限制和RPY偏移量
    if (!ParseJointLimitsAndOffsets()) {
      LE_LOG_ERROR << "Failed to parse joint limits and offsets from URDF"
                   << std::endl;
      return false;
    }

    LE_LOG_INFO << "Successfully loaded joint limits from URDF content"
                << std::endl;
    return true;

  } catch (const std::exception &e) {
    LE_LOG_ERROR << "Exception while parsing URDF content: " << e.what()
                 << std::endl;
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

  // Initialize tip link transform
  tip_link_initial_transform_.setIdentity();

  // 设置默认关节RPY偏移量（基于URDF分析和关节轴向得出的默认值）
  // 顺序必须与joint_names_一致：["shoulder_pan", "shoulder_lift", "elbow_flex",
  // "wrist_flex", "wrist_roll"] shoulder_pan: rpy="3.14159 4.18253e-17
  // -3.14159" -> 绕Z轴，使用yaw=-π shoulder_lift: rpy="-1.5708 -1.5708 0" ->
  // 局部Z轴转为Y轴方向，使用roll=-π/2 elbow_flex:
  // rpy="-3.63608e-16 8.74301e-16 1.5708" -> 绕Z轴，使用yaw=π/2 wrist_flex:
  // rpy="4.02456e-15 8.67362e-16 -1.5708" -> 绕Z轴，使用yaw=-π/2 wrist_roll:
  // rpy="1.5708 0.0486795 3.14159" -> 绕Z轴，使用yaw=π
  joint_origin_rpy_offsets_ = {-M_PI, -M_PI_2, M_PI_2, -M_PI_2, M_PI};
}

void XLeRobotKinematics::PrintArmLengthParameters() const {
  LE_LOG_INFO << "=== Arm Length Parameters ===" << std::endl;
  LE_LOG_INFO << "base_to_shoulder_pan_height_: " << std::fixed << std::setprecision(5) 
              << base_to_shoulder_pan_height_ << std::endl;
  LE_LOG_INFO << "base_to_shoulder_lift_height_: " << std::fixed
              << std::setprecision(5) << base_to_shoulder_lift_height_
              << std::endl;
  LE_LOG_INFO << "shoulder_pan_offset_: " << std::fixed << std::setprecision(5) 
              << shoulder_pan_offset_ << std::endl;
  LE_LOG_INFO << "upper_arm_length_: " << std::fixed << std::setprecision(5) 
              << upper_arm_length_ << std::endl;
  LE_LOG_INFO << "lower_arm_length_: " << std::fixed << std::setprecision(5) 
              << lower_arm_length_ << std::endl;
  LE_LOG_INFO << "wrist_flex_to_roll_offset_: " << std::fixed << std::setprecision(5) 
              << wrist_flex_to_roll_offset_ << std::endl;
  LE_LOG_INFO << "wrist_roll_to_gripper_offset_: " << std::fixed << std::setprecision(5) 
              << wrist_roll_to_gripper_offset_ << std::endl;
  LE_LOG_INFO << "min_arm_reach_: " << std::fixed << std::setprecision(5)
              << min_arm_reach_ << std::endl;
  LE_LOG_INFO << "max_arm_reach_: " << std::fixed << std::setprecision(5)
              << max_arm_reach_ << std::endl;
  LE_LOG_INFO << "=============================" << std::endl;
}

void XLeRobotKinematics::ParseWorkspaceParameters() {
  min_arm_reach_ = std::abs(upper_arm_length_ - lower_arm_length_);
  max_arm_reach_ = upper_arm_length_ + lower_arm_length_;
}

bool XLeRobotKinematics::ParseKinematicsParameters() {
  // 常量定义
  constexpr double kLengthCompensation = 0.03; // 下臂长度补偿（3cm）

  if (!urdf_model_) {
    LE_LOG_ERROR << "URDF model not loaded" << std::endl;
    return false;
  }

  try {
    // 1. 基座到shoulder_pan的高度
    auto shoulder_pan_joint = urdf_model_->getJoint("shoulder_pan");
    if (shoulder_pan_joint) {
      double z_offset =
          shoulder_pan_joint->parent_to_joint_origin_transform.position.z;
      if (z_offset > 0.001) { // 有效高度阈值
        base_to_shoulder_pan_height_ = z_offset;
        LE_LOG_INFO << "Parsed base_to_shoulder_pan_height from URDF: "
                    << std::fixed << std::setprecision(5)
                    << base_to_shoulder_pan_height_ << std::endl;
      } else {
        LE_LOG_ERROR << "Invalid base_to_shoulder_pan_height from URDF ("
                     << std::fixed << std::setprecision(5) << z_offset
                     << "), using default value: "
                     << base_to_shoulder_pan_height_ << std::endl;
      }
    } else {
      LE_LOG_ERROR << "shoulder_pan joint not found in URDF, using default "
                      "base_to_shoulder_pan_height: "
                   << std::fixed << std::setprecision(5)
                   << base_to_shoulder_pan_height_ << std::endl;
    }

    // 2. 基座到shoulder_lift的高度
    auto shoulder_lift_joint = urdf_model_->getJoint("shoulder_lift");
    if (shoulder_lift_joint) {
      double z_offset =
          shoulder_lift_joint->parent_to_joint_origin_transform.position.z;
      if (z_offset > 0.001) { // 有效高度阈值
        base_to_shoulder_lift_height_ = z_offset;
        LE_LOG_INFO << "Parsed base_to_shoulder_lift_height from URDF: "
                    << std::fixed << std::setprecision(5)
                    << base_to_shoulder_lift_height_ << std::endl;
      } else {
        LE_LOG_ERROR << "Invalid base_to_shoulder_lift_height from URDF ("
                     << std::fixed << std::setprecision(5) << z_offset
                     << "), using default value: "
                     << base_to_shoulder_lift_height_ << std::endl;
      }
    } else {
      LE_LOG_ERROR << "shoulder_lift joint not found in URDF, using default "
                      "base_to_shoulder_lift_height: "
                   << std::fixed << std::setprecision(5)
                   << base_to_shoulder_lift_height_ << std::endl;
    }

    // 3. 上臂长度：从shoulder_lift到elbow_flex的距离
    auto elbow_flex_joint = urdf_model_->getJoint("elbow_flex");
    if (elbow_flex_joint) {
      // 通过elbow_flex关节的偏移获取上臂长度
      double x_offset =
          elbow_flex_joint->parent_to_joint_origin_transform.position.x;
      double y_offset =
          elbow_flex_joint->parent_to_joint_origin_transform.position.y;
      double z_offset =
          elbow_flex_joint->parent_to_joint_origin_transform.position.z;
      double parsed_upper_arm_length = std::sqrt(
          x_offset * x_offset + y_offset * y_offset + z_offset * z_offset);

      if (parsed_upper_arm_length > 0.001) { // 有效长度阈值
        upper_arm_length_ = parsed_upper_arm_length + kLengthCompensation;
        LE_LOG_INFO << "Parsed upper_arm_length from URDF: " << std::fixed
                    << std::setprecision(5) << upper_arm_length_ << std::endl;
      } else {
        LE_LOG_ERROR << "Invalid upper_arm_length from URDF (" << std::fixed
                     << std::setprecision(5) << parsed_upper_arm_length
                     << "), using default value: " << upper_arm_length_
                     << std::endl;
      }
    } else {
      LE_LOG_ERROR << "elbow_flex joint not found in URDF, using default "
                      "upper_arm_length: "
                   << std::fixed << std::setprecision(5) << upper_arm_length_
                   << std::endl;
    }

    // 4. 下臂长度：从elbow_flex到wrist_flex的距离
    auto wrist_flex_joint = urdf_model_->getJoint("wrist_flex");
    if (wrist_flex_joint) {
      // 通过wrist_flex关节的偏移获取下臂长度
      double x_offset =
          wrist_flex_joint->parent_to_joint_origin_transform.position.x;
      double y_offset =
          wrist_flex_joint->parent_to_joint_origin_transform.position.y;
      double z_offset =
          wrist_flex_joint->parent_to_joint_origin_transform.position.z;
      double parsed_lower_arm_length = std::sqrt(
          x_offset * x_offset + y_offset * y_offset + z_offset * z_offset);

      if (parsed_lower_arm_length > 0.001) { // 有效长度阈值
        lower_arm_length_ = parsed_lower_arm_length +
                            kLengthCompensation; // 添加3cm补偿来弥补URDF精度
        LE_LOG_INFO << "Parsed lower_arm_length from URDF: " << std::fixed
                    << std::setprecision(5) << (parsed_lower_arm_length)
                    << ", with 3cm compensation: " << lower_arm_length_
                    << std::endl;
      } else {
        LE_LOG_ERROR << "Invalid lower_arm_length from URDF (" << std::fixed
                     << std::setprecision(5) << parsed_lower_arm_length
                     << "), using default value: " << lower_arm_length_
                     << std::endl;
      }
    } else {
      LE_LOG_ERROR << "wrist_flex joint not found in URDF, using default "
                      "lower_arm_length: "
                   << std::fixed << std::setprecision(5) << lower_arm_length_
                   << std::endl;
    }

    // 5. 腕部偏移：从wrist_flex到wrist_roll的距离
    auto wrist_roll_joint = urdf_model_->getJoint("wrist_roll");
    if (wrist_flex_joint && wrist_roll_joint) {
      double x_offset =
          wrist_roll_joint->parent_to_joint_origin_transform.position.x;
      double y_offset =
          wrist_roll_joint->parent_to_joint_origin_transform.position.y;
      double z_offset =
          wrist_roll_joint->parent_to_joint_origin_transform.position.z;
      wrist_flex_to_roll_offset_ = std::sqrt(
          x_offset * x_offset + y_offset * y_offset + z_offset * z_offset);
      LE_LOG_INFO << "Parsed wrist_flex_to_roll_offset from URDF: "
                  << std::fixed << std::setprecision(5)
                  << wrist_flex_to_roll_offset_ << std::endl;
    }

    // 6. 夹爪偏移：从wrist_roll到gripper_link的距离
    auto gripper_link = urdf_model_->getLink("gripper_link");
    if (wrist_roll_joint && gripper_link) {
      // 通过关节链查找gripper_link的父关节
      // 这里需要更复杂的逻辑来遍历关节链，暂时保持默认值
      LE_LOG_INFO << "Using default wrist_roll_to_gripper_offset: "
                  << std::fixed << std::setprecision(5)
                  << wrist_roll_to_gripper_offset_ << std::endl;
    }

    LE_LOG_INFO << "Successfully parsed kinematics parameters from URDF"
                << std::endl;

    // 更新工作空间参数
    ParseWorkspaceParameters();

    return true;

  } catch (const std::exception &e) {
    LE_LOG_ERROR << "Exception while parsing kinematics parameters: "
                 << e.what() << std::endl;
    return false;
  }
}

bool XLeRobotKinematics::ParseTipLinkInitialOrientation(
    const std::string &base_link, const std::string &tip_link) {
  tip_link_initial_transform_.setIdentity();

  if (!urdf_model_) {
    LE_LOG_ERROR << "URDF model not loaded" << std::endl;
    return false;
  }

  if (tip_link.empty()) {
    LE_LOG_ERROR << "Tip link name is empty" << std::endl;
    return false;
  }

  auto link = urdf_model_->getLink(tip_link);
  if (!link) {
    LE_LOG_ERROR << "Tip link '" << tip_link << "' not found in URDF"
                 << std::endl;
    return false;
  }

  tf2::Quaternion accumulated_orientation(0.0, 0.0, 0.0, 1.0);

  while (link && link->parent_joint) {
    const auto joint = link->parent_joint;
    if (!joint) {
      break;
    }

    const auto &origin_rotation =
        joint->parent_to_joint_origin_transform.rotation;
    tf2::Quaternion joint_orientation(origin_rotation.x, origin_rotation.y,
                                      origin_rotation.z, origin_rotation.w);
    accumulated_orientation = joint_orientation * accumulated_orientation;

    const std::string &parent_link_name = joint->parent_link_name;

    if (!base_link.empty() && parent_link_name == base_link) {
      break;
    }

    link = urdf_model_->getLink(parent_link_name);
    if (!link) {
      if (!base_link.empty()) {
        LE_LOG_ERROR << "Parent link '" << parent_link_name
                     << "' not found while parsing tip link orientation"
                     << std::endl;
        return false;
      }
      break;
    }
  }

  accumulated_orientation.normalize();

  tip_link_initial_transform_.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
  tip_link_initial_transform_.setRotation(accumulated_orientation);

  double initial_roll = 0.0;
  double initial_pitch = 0.0;
  double initial_yaw = 0.0;
  tf2::Matrix3x3(accumulated_orientation)
      .getRPY(initial_roll, initial_pitch, initial_yaw);

  LE_LOG_INFO << "Tip link initial orientation (RPY): roll=" << std::fixed
              << std::setprecision(5) << initial_roll
              << ", pitch=" << initial_pitch << ", yaw=" << initial_yaw
              << std::endl;

  return true;
}

bool XLeRobotKinematics::ParseEeFrameInitialOrientation(
    const std::string &base_link) {
  ee_link_initial_transform_.setIdentity();

  if (end_effector_frame_.empty()) {
    LE_LOG_ERROR << "end_effector_frame_ is empty" << std::endl;
    return false;
  }

  if (!urdf_model_) {
    LE_LOG_ERROR << "URDF model not loaded" << std::endl;
    return false;
  }

  auto link = urdf_model_->getLink(end_effector_frame_);
  if (!link) {
    LE_LOG_ERROR << "End effector link '" << end_effector_frame_
                 << "' not found in URDF" << std::endl;
    return false;
  }

  tf2::Quaternion accumulated_orientation(0.0, 0.0, 0.0, 1.0);

  while (link && link->parent_joint) {
    const auto joint = link->parent_joint;
    if (!joint)
      break;

    const auto &origin_rotation =
        joint->parent_to_joint_origin_transform.rotation;
    tf2::Quaternion joint_orientation(origin_rotation.x, origin_rotation.y,
                                      origin_rotation.z, origin_rotation.w);
    accumulated_orientation = joint_orientation * accumulated_orientation;

    const std::string &parent_link_name = joint->parent_link_name;
    if (!base_link.empty() && parent_link_name == base_link)
      break;

    link = urdf_model_->getLink(parent_link_name);
    if (!link) {
      if (!base_link.empty()) {
        LE_LOG_ERROR << "Parent link '" << parent_link_name
                     << "' not found while parsing EE orientation" << std::endl;
        return false;
      }
      break;
    }
  }

  if (accumulated_orientation.length2() <=
      std::numeric_limits<double>::epsilon()) {
    accumulated_orientation.setValue(0.0, 0.0, 0.0, 1.0);
  }
  accumulated_orientation.normalize();

  ee_link_initial_transform_.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
  ee_link_initial_transform_.setRotation(accumulated_orientation);

  double initial_roll = 0.0;
  double initial_pitch = 0.0;
  double initial_yaw = 0.0;
  tf2::Matrix3x3(accumulated_orientation)
      .getRPY(initial_roll, initial_pitch, initial_yaw);
  LE_LOG_INFO << "EE initial orientation (RPY): roll=" << std::fixed
              << std::setprecision(5) << initial_roll
              << ", pitch=" << initial_pitch << ", yaw=" << initial_yaw
              << std::endl;

  return true;
}

bool XLeRobotKinematics::ParseJointLimitsAndOffsets() {
  if (!urdf_model_) {
    LE_LOG_ERROR << "URDF model not loaded" << std::endl;
    return false;
  }

  try {
    // 初始化关节限制和偏移量向量
    joint_lower_limits_.resize(num_joints_);
    joint_upper_limits_.resize(num_joints_);
    joint_origin_rpy_offsets_.resize(num_joints_);

    for (size_t i = 0; i < joint_names_.size(); ++i) {
      const std::string &joint_name = joint_names_[i];
      auto joint = urdf_model_->getJoint(joint_name);

      if (!joint) {
        LE_LOG_ERROR << "Joint '" << joint_name << "' not found in URDF content"
                     << std::endl;
        return false;
      }

      if (joint->type == urdf::Joint::REVOLUTE ||
          joint->type == urdf::Joint::CONTINUOUS) {
        if (joint->limits) {
          joint_lower_limits_[i] = joint->limits->lower;
          joint_upper_limits_[i] = joint->limits->upper;
          LE_LOG_INFO << "Joint '" << joint_name << "' limits: [" << std::fixed
                      << std::setprecision(5) << joint_lower_limits_[i] << ", "
                      << joint_upper_limits_[i] << "]" << std::endl;
        } else {
          LE_LOG_ERROR << "Joint '" << joint_name
                       << "' has no limits defined in URDF" << std::endl;
          return false;
        }
      } else {
        LE_LOG_ERROR << "Joint '" << joint_name
                     << "' is not a revolute joint (type: " << joint->type
                     << ")" << std::endl;
        return false;
      }
    }

    // 使用硬编码的关节origin RPY偏移量（单位：弧度）
    // 0: 0°, 1: 180°, 2: 90°, 3: 0°, 4: 0°
    if (joint_origin_rpy_offsets_.size() < 5) {
      LE_LOG_ERROR << "joint_origin_rpy_offsets_ size is invalid: "
                   << joint_origin_rpy_offsets_.size() << std::endl;
      return false;
    }
    joint_origin_rpy_offsets_[0] = 0.0;
    joint_origin_rpy_offsets_[1] = 0.0;
    joint_origin_rpy_offsets_[2] = M_PI_2;
    joint_origin_rpy_offsets_[3] = 0.0;
    joint_origin_rpy_offsets_[4] = 0.0;

    LE_LOG_INFO << "Using hardcoded joint origin RPY offsets (rad): ["
                << std::fixed << std::setprecision(5)
                << joint_origin_rpy_offsets_[0] << ", "
                << joint_origin_rpy_offsets_[1] << ", "
                << joint_origin_rpy_offsets_[2] << ", "
                << joint_origin_rpy_offsets_[3] << ", "
                << joint_origin_rpy_offsets_[4] << "]" << std::endl;

    return true;

  } catch (const std::exception &e) {
    LE_LOG_ERROR << "Exception while parsing joint limits and offsets: "
                 << e.what() << std::endl;
    return false;
  }
}

} // namespace lerobot_vr_controller
