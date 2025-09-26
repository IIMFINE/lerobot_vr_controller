#include "xlerobot_kinematics.h"

#include "log.h"

#include <iomanip>

namespace lerobot_vr_controller {

XLeRobotKinematics::XLeRobotKinematics()
    : base_to_shoulder_pan_height_(0.0624), shoulder_pan_offset_(0.0),
      upper_arm_length_(0.11257), lower_arm_length_(0.1349),
      wrist_flex_to_roll_offset_(0.0611), wrist_roll_to_gripper_offset_(0.0181),
      urdf_model_(nullptr), position_tolerance_(1e-4),
      orientation_tolerance_(1e-3), num_joints_(5), initialized_(false) {
  // 初始化关节RPY偏移量（默认值，将从URDF更新）
  joint_origin_rpy_offsets_.resize(5, 0.0);
}

bool XLeRobotKinematics::Initialize(const std::string &urdf_param) {

  // 初始化关节名称（SO-ARM101标准关节名称，与URDF一致）
  joint_names_ = {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex",
                  "wrist_roll"};

  // 构建关节占位映射
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    joint_placehold_map_[joint_names_[i]] = i;
  }

  // 从URDF内容字符串加载关节限制
  if (!urdf_param.empty() && !LoadURDF(urdf_param)) {
    LE_LOG_ERROR
        << "Failed to parse URDF content string, using default joint limits"
        << std::endl;
    // 使用默认关节限制作为后备
    SetDefaultJointLimits();
  } else if (urdf_param.empty()) {
    LE_LOG_INFO << "No URDF content provided, using default joint limits"
                << std::endl;
    // 没有提供URDF内容，使用默认限制
    SetDefaultJointLimits();
  } else {
    // URDF成功加载，解析机械臂结构参数
    if (!ParseKinematicsParameters()) {
      LE_LOG_ERROR << "Failed to parse kinematics parameters from URDF, using "
                      "default values"
                   << std::endl;
    }
  }
  initialized_ = true;
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

  // 步骤1：计算第1关节角度（极坐标旋转角）
  // 考虑URDF中shoulder_pan的origin偏移
  double shoulder_pan_angle = std::atan2(y, x) - joint_origin_rpy_offsets_[0];

  // 步骤2：计算垂直平面内的径向距离
  double r = std::sqrt(x * x + y * y);
  double z_relative = z - base_to_shoulder_pan_height_; // 相对于基座的高度

  // 步骤3：使用余弦定理求解第2、3关节角度，考虑URDF偏移
  double shoulder_lift_angle, elbow_flex_angle;
  if (!SolveVerticalPlane2D(r, z_relative, shoulder_lift_angle,
                            elbow_flex_angle)) {
    LE_LOG_ERROR << "Cannot reach target position (r=" << r
                 << ", z=" << z_relative << ")" << std::endl;
    return false;
  }

  // 应用URDF origin偏移
  shoulder_lift_angle -= joint_origin_rpy_offsets_[1];
  elbow_flex_angle -= joint_origin_rpy_offsets_[2];

  // 步骤4：计算末端朝向角度，考虑URDF偏移
  auto [wrist_flex_angle, wrist_roll_angle] = CalculateEndEffectorOrientation(
      target_transform, shoulder_lift_angle + joint_origin_rpy_offsets_[1],
      elbow_flex_angle + joint_origin_rpy_offsets_[2]);

  // 应用腕部关节的URDF偏移
  wrist_flex_angle -= joint_origin_rpy_offsets_[3];
  wrist_roll_angle -= joint_origin_rpy_offsets_[4];

  // 步骤5：直接使用计算得到的关节位置（已经考虑了URDF偏移）
  // 这样确保关节位置为0时，机械臂呈现URDF中定义的几何配置
  solution[0] = shoulder_pan_angle;  // shoulder_pan
  solution[1] = shoulder_lift_angle; // shoulder_lift
  solution[2] = elbow_flex_angle;    // elbow_flex
  solution[3] = wrist_flex_angle;    // wrist_flex
  solution[4] = wrist_roll_angle;    // wrist_roll

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
                                              double &shoulder_lift_angle,
                                              double &elbow_flex_angle) {
  // 计算目标点到第2关节的距离
  double target_distance = std::sqrt(r * r + z * z);

  // 检查是否在工作空间内
  double min_reach = std::abs(upper_arm_length_ - lower_arm_length_);
  double max_reach = upper_arm_length_ + lower_arm_length_;

  if (target_distance < min_reach || target_distance > max_reach) {
    return false;
  }

  // 使用余弦定理求解肘部弯曲角度
  double cos_elbow_flex = (upper_arm_length_ * upper_arm_length_ +
                           lower_arm_length_ * lower_arm_length_ -
                           target_distance * target_distance) /
                          (2 * upper_arm_length_ * lower_arm_length_);

  // 确保cos值在有效范围内
  cos_elbow_flex = std::max(-1.0, std::min(1.0, cos_elbow_flex));

  // 计算肘部弯曲角度（选择正角度解，确保关节角度>0的约束）
  elbow_flex_angle = std::acos(cos_elbow_flex);

  // 计算肩部抬升角度
  double alpha = std::atan2(z, r); // 目标点相对于水平面的角度
  double beta = std::acos((upper_arm_length_ * upper_arm_length_ +
                           target_distance * target_distance -
                           lower_arm_length_ * lower_arm_length_) /
                          (2 * upper_arm_length_ * target_distance));

  shoulder_lift_angle = alpha - beta;

  return true;
}

std::pair<double, double> XLeRobotKinematics::CalculateEndEffectorOrientation(
    const tf2::Transform &target_transform, double shoulder_lift_angle,
    double elbow_flex_angle) {

  // 获取目标方向
  tf2::Matrix3x3 target_rotation(target_transform.getRotation());
  double target_roll, target_pitch, target_yaw;
  target_rotation.getRPY(target_roll, target_pitch, target_yaw);

  // 计算前两个关节造成的累积俯仰角
  double accumulated_pitch = shoulder_lift_angle + elbow_flex_angle;

  // 第4关节直接对应俯仰角，需要补偿前面关节的影响
  double wrist_flex_angle = target_pitch - accumulated_pitch;

  // 第5关节直接对应横滚角
  double wrist_roll_angle = target_roll;

  return std::make_pair(wrist_flex_angle, wrist_roll_angle);
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
  double z_relative = z - base_to_shoulder_pan_height_;
  double target_distance = std::sqrt(r * r + z_relative * z_relative);

  double min_reach = std::abs(upper_arm_length_ - lower_arm_length_);
  double max_reach = upper_arm_length_ + lower_arm_length_;

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

        // 获取关节origin的RPY偏移量
        // 对于绕Z轴旋转的关节，需要根据origin的RPY变换来确定实际的旋转偏移
        if (joint->parent_to_joint_origin_transform.rotation.x != 0.0 ||
            joint->parent_to_joint_origin_transform.rotation.y != 0.0 ||
            joint->parent_to_joint_origin_transform.rotation.z != 0.0 ||
            joint->parent_to_joint_origin_transform.rotation.w != 1.0) {
          // 从四元数转换为RPY
          double roll, pitch, yaw;
          joint->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch,
                                                                  yaw);

          // 分析关节的实际旋转轴方向
          // 所有关节都定义为绕Z轴旋转，但由于origin的RPY变换，
          // 实际的旋转效果需要考虑坐标系变换
          if (joint_name == "shoulder_pan") {
            // shoulder_pan: 绕世界坐标系Z轴旋转，但有yaw偏移
            joint_origin_rpy_offsets_[i] = yaw;
          } else if (joint_name == "shoulder_lift") {
            // shoulder_lift: 由于rpy="-1.5708 -1.5708
            // 0"，局部Z轴变为父坐标系的Y轴 这种情况下主要是roll偏移影响
            joint_origin_rpy_offsets_[i] = roll;
          } else if (joint_name == "elbow_flex") {
            // elbow_flex: rpy包含yaw=π/2的偏移
            joint_origin_rpy_offsets_[i] = yaw;
          } else if (joint_name == "wrist_flex") {
            // wrist_flex: rpy包含yaw=-π/2的偏移
            joint_origin_rpy_offsets_[i] = yaw;
          } else if (joint_name == "wrist_roll") {
            // wrist_roll: 主要是yaw=π的偏移
            joint_origin_rpy_offsets_[i] = yaw;
          } else {
            // 默认情况，使用yaw偏移
            joint_origin_rpy_offsets_[i] = yaw;
          }

          LE_LOG_INFO << "Joint '" << joint_name << "' origin RPY: ["
                      << std::fixed << std::setprecision(5) << roll << ", "
                      << pitch << ", " << yaw
                      << "] -> using offset=" << joint_origin_rpy_offsets_[i]
                      << std::endl;
        } else {
          joint_origin_rpy_offsets_[i] = 0.0;
        }
      } else {
        LE_LOG_ERROR << "Joint '" << joint_name
                     << "' is not a revolute joint (type: " << joint->type
                     << ")" << std::endl;
        return false;
      }
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

  // 设置默认关节RPY偏移量（基于URDF分析和关节轴向得出的默认值）
  // 顺序必须与joint_names_一致：["shoulder_pan", "shoulder_lift", "elbow_flex",
  // "wrist_flex", "wrist_roll"] shoulder_pan: rpy="3.14159 4.18253e-17
  // -3.14159" -> 绕Z轴，使用yaw=-π shoulder_lift: rpy="-1.5708 -1.5708 0" ->
  // 局部Z轴转为Y轴方向，使用roll=-π/2 elbow_flex:
  // rpy="-3.63608e-16 8.74301e-16 1.5708" -> 绕Z轴，使用yaw=π/2 wrist_flex:
  // rpy="4.02456e-15 8.67362e-16 -1.5708" -> 绕Z轴，使用yaw=-π/2 wrist_roll:
  // rpy="1.5708 0.0486795 3.14159" -> 绕Z轴，使用yaw=π
  joint_origin_rpy_offsets_ = {-M_PI, -M_PI_2, M_PI_2, -M_PI_2, M_PI};

  LE_LOG_INFO << "Using default joint limits and RPY offsets" << std::endl;
}

bool XLeRobotKinematics::ParseKinematicsParameters() {
  if (!urdf_model_) {
    LE_LOG_ERROR << "URDF model not loaded" << std::endl;
    return false;
  }

  try {
    // 解析机械臂结构参数
    // 这些参数通过分析URDF中链接的几何关系来获取

    // 1. 基座高度：从base_link到shoulder_pan_link的距离
    auto base_link = urdf_model_->getLink("base_link");
    auto shoulder_pan_link = urdf_model_->getLink("shoulder_pan_link");
    if (base_link && shoulder_pan_link) {
      // 通过关节信息获取高度偏移
      auto shoulder_pan_joint = urdf_model_->getJoint("shoulder_pan");
      if (shoulder_pan_joint &&
          shoulder_pan_joint->parent_to_joint_origin_transform.position.z !=
              0) {
        base_to_shoulder_pan_height_ =
            shoulder_pan_joint->parent_to_joint_origin_transform.position.z;
        LE_LOG_INFO << "Parsed base_to_shoulder_pan_height from URDF: "
                    << std::fixed << std::setprecision(5)
                    << base_to_shoulder_pan_height_ << std::endl;
      }
    }

    // 2. 上臂长度：从shoulder_lift到elbow_flex的距离
    auto shoulder_lift_joint = urdf_model_->getJoint("shoulder_lift");
    auto elbow_flex_joint = urdf_model_->getJoint("elbow_flex");
    if (shoulder_lift_joint && elbow_flex_joint) {
      // 获取上臂链接
      auto upper_arm_link = urdf_model_->getLink("upper_arm_link");
      if (upper_arm_link) {
        // 通过elbow_flex关节的偏移获取上臂长度
        double x_offset =
            elbow_flex_joint->parent_to_joint_origin_transform.position.x;
        double y_offset =
            elbow_flex_joint->parent_to_joint_origin_transform.position.y;
        double z_offset =
            elbow_flex_joint->parent_to_joint_origin_transform.position.z;
        upper_arm_length_ = std::sqrt(
            x_offset * x_offset + y_offset * y_offset + z_offset * z_offset);
        LE_LOG_INFO << "Parsed upper_arm_length from URDF: " << std::fixed
                    << std::setprecision(5) << upper_arm_length_ << std::endl;
      }
    }

    // 3. 下臂长度：从elbow_flex到wrist_flex的距离
    auto wrist_flex_joint = urdf_model_->getJoint("wrist_flex");
    if (elbow_flex_joint && wrist_flex_joint) {
      // 获取下臂链接
      auto lower_arm_link = urdf_model_->getLink("lower_arm_link");
      if (lower_arm_link) {
        // 通过wrist_flex关节的偏移获取下臂长度
        double x_offset =
            wrist_flex_joint->parent_to_joint_origin_transform.position.x;
        double y_offset =
            wrist_flex_joint->parent_to_joint_origin_transform.position.y;
        double z_offset =
            wrist_flex_joint->parent_to_joint_origin_transform.position.z;
        lower_arm_length_ = std::sqrt(
            x_offset * x_offset + y_offset * y_offset + z_offset * z_offset);
        LE_LOG_INFO << "Parsed lower_arm_length from URDF: " << std::fixed
                    << std::setprecision(5) << lower_arm_length_ << std::endl;
      }
    }

    // 4. 腕部偏移：从wrist_flex到wrist_roll的距离
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

    // 5. 夹爪偏移：从wrist_roll到gripper_link的距离
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
    return true;

  } catch (const std::exception &e) {
    LE_LOG_ERROR << "Exception while parsing kinematics parameters: "
                 << e.what() << std::endl;
    return false;
  }
}

} // namespace lerobot_vr_controller
