#include "kinematics.h"
#include "log.h"

#include <iostream>
#include <kdl_parser/kdl_parser.hpp>
#include <mutex>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_kdl/tf2_kdl.hpp>

namespace lerobot_vr_controller {

SoArm101Kinematics::SoArm101Kinematics()
    : timeout_(0.01), position_tolerance_(0.02), orientation_tolerance_(0.8),
      num_joints_(0), initialized_(false), last_solution_cache_(0) {
  LE_LOG_INFO << "SoArm101Kinematics initialized with default tolerances"
              << std::endl;
}

bool SoArm101Kinematics::Initialize(const std::string &urdf_string,
                                    const std::string &base_link,
                                    const std::string &tip_link,
                                    double timeout) {
  base_link_ = base_link;
  tip_link_ = tip_link;
  timeout_ = timeout;

  // 加载URDF模型
  if (!LoadURDF(urdf_string)) {
    LE_LOG_ERROR << "Failed to load URDF model" << std::endl;
    return false;
  }

  // 从URDF构建KDL运动学链
  if (!kdl_parser::treeFromUrdfModel(*urdf_model_, kdl_tree_)) {
    LE_LOG_ERROR << "Failed to construct KDL tree from URDF" << std::endl;
    return false;
  }

  if (!kdl_tree_.getChain(base_link_, tip_link_, kinematic_chain_)) {
    LE_LOG_ERROR << "Failed to get kinematic chain from " << base_link_
                 << " to " << tip_link_ << std::endl;
    return false;
  }

  num_joints_ = kinematic_chain_.getNrOfJoints();
  if (num_joints_ == 0) {
    LE_LOG_ERROR << "No joints found in kinematic chain" << std::endl;
    return false;
  }

  LE_LOG_INFO << "Kinematic chain has " << num_joints_ << " joints"
              << std::endl;

  // 初始化TRAC-IK求解器，使用优化配置
  trac_ik_solver_ = std::make_unique<TRAC_IK::TRAC_IK>(
      base_link_, tip_link_, urdf_string, timeout_,
      1e-5, // 略放宽公差以减少迭代次数
      TRAC_IK::Distance);

  if (!trac_ik_solver_->getKDLChain(kinematic_chain_)) {
    LE_LOG_ERROR << "Failed to get KDL chain from TRAC-IK" << std::endl;
    return false;
  }

  // 获取关节限制和名称
  KDL::JntArray lower_limits, upper_limits;
  if (!trac_ik_solver_->getKDLLimits(lower_limits, upper_limits)) {
    LE_LOG_ERROR << "Failed to get joint limits" << std::endl;
    return false;
  }

  joint_lower_limits_.resize(num_joints_);
  joint_upper_limits_.resize(num_joints_);
  joint_names_.clear();
  joint_placehold_map_.clear();

  for (size_t i = 0; i < num_joints_; ++i) {
    joint_lower_limits_[i] = lower_limits(i);
    joint_upper_limits_[i] = upper_limits(i);

    // 从运动学链获取关节名称
    const KDL::Segment &segment = kinematic_chain_.getSegment(i);
    const KDL::Joint &joint = segment.getJoint();
    if (joint.getType() != KDL::Joint::None) {
      joint_names_.push_back(joint.getName());
      joint_placehold_map_[joint.getName()] = static_cast<int>(i);
    }
  }

  initialized_ = true;

  // 初始化缓存
  last_solution_cache_.resize(num_joints_, 0.0);

  // 生成预定义种子策略
  GeneratePredefinedSeeds();

  LE_LOG_INFO << "SoArm101Kinematics initialized successfully with "
              << num_joints_ << " joints and " << predefined_seeds_.size()
              << " predefined seeds" << std::endl;

  // 打印所有关节名称
  LE_LOG_INFO << "Joint names: [";
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    LE_LOG_INFO << "\"" << joint_names_[i] << "\"";
    if (i < joint_names_.size() - 1) {
      LE_LOG_INFO << ", ";
    }
  }
  LE_LOG_INFO << "]" << std::endl;

  return true;
}

bool SoArm101Kinematics::SolveIK(const tf2::Transform &target_transform,
                                 std::vector<double> &solution,
                                 const std::vector<double> &seed_joints) {
  if (!initialized_) {
    LE_LOG_ERROR << "Failed to solve IK: Kinematics solver not initialized"
                 << std::endl;
    return false;
  }

  // 转换为KDL数据类型
  KDL::Frame target_frame = TransformToKDLFrame(target_transform);

  // 快速工作空间检查 (放宽限制)
  if (!CheckWorkspace(target_transform)) {
    return false; // 直接返回，不打印错误信息
  }

  std::vector<KDL::JntArray> seed_candidates;

  // 优先使用上次成功的解作为种子
  {
    std::lock_guard<std::mutex> lock(solution_cache_mutex_);
    if (!last_solution_cache_.empty()) {
      KDL::JntArray cached_seed(num_joints_);
      for (size_t i = 0; i < num_joints_; ++i) {
        cached_seed(i) = last_solution_cache_[i];
      }
      seed_candidates.push_back(cached_seed);
    }
  }

  // 生成智能种子
  std::vector<std::vector<double>> smart_seeds;
  GenerateSmartSeeds(target_transform, seed_joints, smart_seeds);

  for (const auto &seed : smart_seeds) {
    KDL::JntArray seed_array(num_joints_);
    for (size_t i = 0; i < num_joints_; ++i) {
      seed_array(i) = seed[i];
    }
    seed_candidates.push_back(seed_array);
  }

  // 限制最大尝试次数以控制计算时间
  size_t max_attempts = std::min(seed_candidates.size(), size_t(6));

  // 尝试每个种子候选
  for (size_t attempt = 0; attempt < max_attempts; ++attempt) {
    KDL::JntArray solution_array(num_joints_);
    if (SolveIK(target_frame, seed_candidates[attempt], solution_array)) {
      // 转换回std::vector
      solution.resize(num_joints_);
      for (size_t i = 0; i < num_joints_; ++i) {
        solution[i] = solution_array(i);
      }

      // 缓存成功的解
      {
        std::lock_guard<std::mutex> lock(solution_cache_mutex_);
        last_solution_cache_ = solution;
      }

      return true;
    }
  }
  LE_LOG_ERROR_T(1s) << "Failed to solve IK for the given target transform"
                     << std::endl;
  return false;
}

bool SoArm101Kinematics::SolveIK(const KDL::Frame &target_frame,
                                 const KDL::JntArray &seed_joints,
                                 KDL::JntArray &solution) {
  if (!initialized_) {
    LE_LOG_ERROR << "Failed to solve IK: Kinematics solver not initialized"
                 << std::endl;
    return false;
  }

  if (static_cast<size_t>(seed_joints.rows()) != num_joints_) {
    LE_LOG_ERROR << "Failed to solve IK: Seed joints array size ("
                 << seed_joints.rows() << ") doesn't match number of joints ("
                 << num_joints_ << ")" << std::endl;
    return false;
  }

  // 设置容忍度：位置容忍度和方向容忍度都使用类成员变量
  KDL::Twist bounds = KDL::Twist::Zero();
  bounds.vel = KDL::Vector(position_tolerance_, position_tolerance_,
                           position_tolerance_);
  bounds.rot = KDL::Vector(orientation_tolerance_, orientation_tolerance_,
                           orientation_tolerance_);

  // 调用TRAC-IK求解
  int result =
      trac_ik_solver_->CartToJnt(seed_joints, target_frame, solution, bounds);

  if (result < 0) {
    return false;
  }

  // 验证解是否在关节限制范围内
  std::vector<double> solution_vec(num_joints_);
  for (size_t i = 0; i < num_joints_; ++i) {
    solution_vec[i] = solution(i);
  }

  ValidateJoints(solution_vec);

  return true;
}

size_t SoArm101Kinematics::GetNumJoints() const { return num_joints_; }

bool SoArm101Kinematics::GetJointLimits(
    std::vector<double> &lower_limits,
    std::vector<double> &upper_limits) const {
  if (!initialized_) {
    return false;
  }

  lower_limits = joint_lower_limits_;
  upper_limits = joint_upper_limits_;
  return true;
}

void SoArm101Kinematics::SetTimeout(double timeout) { timeout_ = timeout; }

void SoArm101Kinematics::SetTolerances(double position_tolerance,
                                       double orientation_tolerance) {
  position_tolerance_ = position_tolerance;
  orientation_tolerance_ = orientation_tolerance;

  LE_LOG_INFO << "Updated IK tolerances - Position: " << position_tolerance_
              << ", Orientation: " << orientation_tolerance_ << std::endl;
}

bool SoArm101Kinematics::IsInitialized() const { return initialized_; }

std::vector<std::string> SoArm101Kinematics::GetJointNames() const {
  return joint_names_;
}

std::map<std::string, int> SoArm101Kinematics::GetJointPlaceholdMap() const {
  if (!initialized_) {
    LE_LOG_ERROR << "Failed to get joint placehold map: Kinematics solver not "
                    "initialized"
                 << std::endl;
    return std::map<std::string, int>();
  }

  return joint_placehold_map_;
}

bool SoArm101Kinematics::AlignJointStateToIk(
    const std::vector<std::string> &joint_names,
    const std::vector<double> &joint_positions,
    std::vector<double> &seed_joints) const {

  if (!initialized_) {
    LE_LOG_ERROR
        << "Failed to align joint state: Kinematics solver not initialized"
        << std::endl;
    return false;
  }

  if (joint_names.size() != joint_positions.size()) {
    LE_LOG_ERROR
        << "Failed to align joint state: Joint names and positions size "
           "mismatch"
        << std::endl;
    return false;
  }

  // 初始化种子关节角度为0
  seed_joints.resize(num_joints_, 0.0);

  // 获取关节映射
  const auto &joint_map = GetJointPlaceholdMap();

  // 将关节状态按照IK求解器的关节顺序排列
  for (size_t i = 0; i < joint_names.size(); ++i) {
    const std::string &joint_name = joint_names[i];
    auto it = joint_map.find(joint_name);

    if (it != joint_map.end()) {
      int index = it->second;
      if (index >= 0 && index < static_cast<int>(num_joints_)) {
        seed_joints[index] = joint_positions[i];
      } else {
        LE_LOG_ERROR << "Failed to align joint state: Invalid joint index "
                     << index << " for joint '" << joint_name << "'"
                     << std::endl;
        return false;
      }
    }
  }

  return true;
}

KDL::Frame
SoArm101Kinematics::TransformToKDLFrame(const tf2::Transform &transform) const {
  KDL::Frame frame;

  // Convert tf2::Transform to KDL::Frame manually
  const tf2::Vector3 &origin = transform.getOrigin();
  const tf2::Quaternion &rotation = transform.getRotation();

  // Set position
  frame.p = KDL::Vector(origin.x(), origin.y(), origin.z());

  // Convert quaternion to rotation matrix
  frame.M = KDL::Rotation::Quaternion(rotation.x(), rotation.y(), rotation.z(),
                                      rotation.w());

  return frame;
}

bool SoArm101Kinematics::ValidateJoints(
    const std::vector<double> &joints) const {
  if (joints.size() != num_joints_) {
    return false;
  }

  bool all_valid = true;
  for (size_t i = 0; i < num_joints_; ++i) {
    if (joints[i] < joint_lower_limits_[i] ||
        joints[i] > joint_upper_limits_[i]) {
      all_valid = false;
    }
  }

  return all_valid;
}

bool SoArm101Kinematics::LoadURDF(const std::string &urdf_string) {
  urdf_model_ = std::make_unique<urdf::Model>();
  if (!urdf_model_->initString(urdf_string)) {
    LE_LOG_ERROR << "Failed to parse URDF string" << std::endl;
    return false;
  }

  return true;
}

bool SoArm101Kinematics::CheckWorkspace(
    const tf2::Transform &target_transform) const {
  if (!initialized_) {
    return false;
  }

  // 转换为KDL数据类型
  KDL::Frame target_frame = TransformToKDLFrame(target_transform);

  // 放宽Z坐标检查
  if (target_frame.p.z() < -0.2) { // 基座以下20cm
    return false;                  // 不打印错误信息
  }

  return true;
}

void SoArm101Kinematics::GenerateSmartSeeds(
    [[maybe_unused]] const tf2::Transform &target_transform,
    const std::vector<double> &current_joints,
    std::vector<std::vector<double>> &smart_seeds) const {
  smart_seeds.clear();

  // 如果有当前关节状态，优先使用
  if (!current_joints.empty() && current_joints.size() == num_joints_) {
    smart_seeds.push_back(current_joints);

    // 生成当前状态的小扰动
    std::vector<double> perturbed = current_joints;
    for (size_t i = 0; i < num_joints_; ++i) {
      perturbed[i] += (i % 2 == 0 ? 0.1 : -0.1); // 小幅度扰动
      // 确保在关节限制内
      perturbed[i] = std::max(joint_lower_limits_[i],
                              std::min(joint_upper_limits_[i], perturbed[i]));
    }
    smart_seeds.push_back(perturbed);
  }
}

void SoArm101Kinematics::GeneratePredefinedSeeds() {
  predefined_seeds_.clear();
  predefined_seeds_.reserve(6);

  if (num_joints_ < 5)
    return;

  // 种子1: 零位
  predefined_seeds_.push_back(std::vector<double>(num_joints_, 0.0));

  // 种子2: 合理的初始姿态
  std::vector<double> reasonable_pose = {0.0, 0.5, -0.3, 0.1, 0.0};
  reasonable_pose.resize(num_joints_, 0.0);
  predefined_seeds_.push_back(reasonable_pose);

  // 种子3: 另一个合理配置
  std::vector<double> reasonable_pose2 = {0.3, 0.8, -0.5, 0.2, 0.0};
  reasonable_pose2.resize(num_joints_, 0.0);
  predefined_seeds_.push_back(reasonable_pose2);

  // 种子4: 左侧配置
  std::vector<double> left_config = {1.57, 0.5, -0.8, 0.3, 0.0};
  left_config.resize(num_joints_, 0.0);
  predefined_seeds_.push_back(left_config);

  // 种子5: 右侧配置
  std::vector<double> right_config = {-1.57, 0.5, -0.8, 0.3, 0.0};
  right_config.resize(num_joints_, 0.0);
  predefined_seeds_.push_back(right_config);

  // 种子6: 中位数配置
  std::vector<double> middle_config(num_joints_);
  for (size_t i = 0; i < num_joints_; ++i) {
    middle_config[i] = (joint_lower_limits_[i] + joint_upper_limits_[i]) / 2.0;
  }
  predefined_seeds_.push_back(middle_config);

  // 确保所有种子都在关节限制内
  for (auto &seed : predefined_seeds_) {
    for (size_t i = 0; i < seed.size() && i < num_joints_; ++i) {
      seed[i] = std::max(joint_lower_limits_[i],
                         std::min(joint_upper_limits_[i], seed[i]));
    }
  }
}

} // namespace lerobot_vr_controller
