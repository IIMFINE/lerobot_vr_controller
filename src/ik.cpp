#include "ik.h"

#include <iostream>
#include <kdl_parser/kdl_parser.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_kdl/tf2_kdl.hpp>

namespace lerobot_vr_controller {

SoArm101Kinematics::SoArm101Kinematics()
    : timeout_(0.005), position_tolerance_(1e-5), orientation_tolerance_(1e-3),
      num_joints_(0), initialized_(false) {
  std::cout << "SoArm101Kinematics initialized" << std::endl;
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
    std::cerr << "Failed to load URDF model" << std::endl;
    return false;
  }

  // 从URDF构建KDL运动学链
  if (!kdl_parser::treeFromUrdfModel(*urdf_model_, kdl_tree_)) {
    std::cerr << "Failed to construct KDL tree from URDF" << std::endl;
    return false;
  }

  if (!kdl_tree_.getChain(base_link_, tip_link_, kinematic_chain_)) {
    std::cerr << "Failed to get kinematic chain from " << base_link_ << " to "
              << tip_link_ << std::endl;
    return false;
  }

  num_joints_ = kinematic_chain_.getNrOfJoints();
  if (num_joints_ == 0) {
    std::cerr << "No joints found in kinematic chain" << std::endl;
    return false;
  }

  // 初始化TRAC-IK求解器
  trac_ik_solver_ = std::make_unique<TRAC_IK::TRAC_IK>(
      base_link_, tip_link_, urdf_string, timeout_, position_tolerance_,
      TRAC_IK::Speed);

  if (!trac_ik_solver_->getKDLChain(kinematic_chain_)) {
    std::cerr << "Failed to get KDL chain from TRAC-IK" << std::endl;
    return false;
  }

  // 获取关节限制和名称
  KDL::JntArray lower_limits, upper_limits;
  if (!trac_ik_solver_->getKDLLimits(lower_limits, upper_limits)) {
    std::cerr << "Failed to get joint limits" << std::endl;
    return false;
  }

  joint_lower_limits_.resize(num_joints_);
  joint_upper_limits_.resize(num_joints_);
  joint_names_.clear();

  for (size_t i = 0; i < num_joints_; ++i) {
    joint_lower_limits_[i] = lower_limits(i);
    joint_upper_limits_[i] = upper_limits(i);

    // 从运动学链获取关节名称
    const KDL::Segment &segment = kinematic_chain_.getSegment(i);
    const KDL::Joint &joint = segment.getJoint();
    if (joint.getType() != KDL::Joint::None) {
      joint_names_.push_back(joint.getName());
    }
  }

  initialized_ = true;
  std::cout << "SoArm101Kinematics initialized successfully with "
            << num_joints_ << " joints" << std::endl;

  return true;
}

bool SoArm101Kinematics::SolveIK(const tf2::Transform &target_transform,
                                 std::vector<double> &solution,
                                 const std::vector<double> &seed_joints) {
  if (!initialized_) {
    std::cerr << "Kinematics solver not initialized" << std::endl;
    return false;
  }

  // 转换为KDL数据类型
  KDL::Frame target_frame = TransformToKDLFrame(target_transform);

  KDL::JntArray seed_array(num_joints_);
  if (seed_joints.empty()) {
    // 使用中间值作为种子
    for (size_t i = 0; i < num_joints_; ++i) {
      seed_array(i) = (joint_lower_limits_[i] + joint_upper_limits_[i]) / 2.0;
    }
  } else {
    if (seed_joints.size() != num_joints_) {
      std::cerr << "Seed joints size (" << seed_joints.size()
                << ") doesn't match number of joints (" << num_joints_ << ")"
                << std::endl;
      return false;
    }
    for (size_t i = 0; i < num_joints_; ++i) {
      seed_array(i) = seed_joints[i];
    }
  }

  KDL::JntArray solution_array(num_joints_);
  if (!SolveIK(target_frame, seed_array, solution_array)) {
    return false;
  }

  // 转换回std::vector
  solution.resize(num_joints_);
  for (size_t i = 0; i < num_joints_; ++i) {
    solution[i] = solution_array(i);
  }

  return true;
}

bool SoArm101Kinematics::SolveIK(const KDL::Frame &target_frame,
                                 const KDL::JntArray &seed_joints,
                                 KDL::JntArray &solution) {
  if (!initialized_) {
    std::cerr << "Kinematics solver not initialized" << std::endl;
    return false;
  }

  if (static_cast<size_t>(seed_joints.rows()) != num_joints_) {
    std::cerr << "Seed joints array size (" << seed_joints.rows()
              << ") doesn't match number of joints (" << num_joints_ << ")"
              << std::endl;
    return false;
  }

  // 调用TRAC-IK求解
  int result = trac_ik_solver_->CartToJnt(seed_joints, target_frame, solution);

  if (result < 0) {
    std::cout << "IK solution not found" << std::endl;
    return false;
  }

  // 验证解是否在关节限制范围内
  std::vector<double> solution_vec(num_joints_);
  for (size_t i = 0; i < num_joints_; ++i) {
    solution_vec[i] = solution(i);
  }

  if (!ValidateJoints(solution_vec)) {
    std::cout << "IK solution violates joint limits" << std::endl;
    return false;
  }

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

void SoArm101Kinematics::SetTimeout(double timeout) {
  timeout_ = timeout;
  if (trac_ik_solver_) {
    // 重新创建求解器以应用新的超时时间
    trac_ik_solver_ = std::make_unique<TRAC_IK::TRAC_IK>(
        base_link_, tip_link_, "robot_description", timeout_,
        position_tolerance_, TRAC_IK::Speed);
  }
}

void SoArm101Kinematics::SetPrecision(double eps, double eps_rot) {
  position_tolerance_ = eps;
  orientation_tolerance_ = eps_rot;

  if (trac_ik_solver_) {
    // 重新创建求解器以应用新的精度
    trac_ik_solver_ = std::make_unique<TRAC_IK::TRAC_IK>(
        base_link_, tip_link_, "robot_description", timeout_,
        position_tolerance_, TRAC_IK::Speed);
  }
}

bool SoArm101Kinematics::IsInitialized() const { return initialized_; }

std::vector<std::string> SoArm101Kinematics::GetJointNames() const {
  return joint_names_;
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

  for (size_t i = 0; i < num_joints_; ++i) {
    if (joints[i] < joint_lower_limits_[i] ||
        joints[i] > joint_upper_limits_[i]) {
      std::cout << "Joint " << i << " value " << joints[i]
                << " is outside limits [" << joint_lower_limits_[i] << ", "
                << joint_upper_limits_[i] << "]" << std::endl;
      return false;
    }
  }

  return true;
}

bool SoArm101Kinematics::LoadURDF(const std::string &urdf_string) {
  urdf_model_ = std::make_unique<urdf::Model>();
  if (!urdf_model_->initString(urdf_string)) {
    std::cerr << "Failed to parse URDF string" << std::endl;
    return false;
  }

  std::cout << "Successfully loaded URDF model" << std::endl;
  return true;
}

} // namespace lerobot_vr_controller
