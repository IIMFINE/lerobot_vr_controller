#ifndef LEROBOT_VR_CONTROLLER_XLEROBOT_KINEMATICS_H_
#define LEROBOT_VR_CONTROLLER_XLEROBOT_KINEMATICS_H_

#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <fstream>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <urdf/model.h>

#include "kinematics.h"

namespace lerobot_vr_controller {

/**
 * @brief 基于极坐标的SoArm101机器人解析逆运动学求解器
 *
 * 使用极坐标系统和解析解，相比传统笛卡尔坐标IK具有以下优势：
 * - 对人类来说更直观，简化键盘控制和VR控制难度
 * - 有利于任务规划，旋转对称性将搜索空间从3D降至2D
 * - 支持对称策略训练，专注于垂直平面操作
 */
class XLeRobotKinematics : public KinematicsInterface {
public:
  /**
   * @brief 构造函数
   */
  XLeRobotKinematics();

  /**
   * @brief 析构函数
   */
  ~XLeRobotKinematics() override = default;

  // 禁用拷贝构造和赋值操作
  XLeRobotKinematics(const XLeRobotKinematics &) = delete;
  XLeRobotKinematics &operator=(const XLeRobotKinematics &) = delete;

  /**
   * @brief 初始化运动学求解器
   * @param urdf_param URDF文件内容字符串（为空则使用默认限制）
   * @param base_link 基座链接名称（未使用，保持接口一致性）
   * @param tip_link 末端链接名称（未使用，保持接口一致性） 
   * @param timeout 求解超时时间（未使用，保持接口一致性）
   * @return 初始化是否成功
   */
  bool Initialize(const std::string &urdf_param,
                  [[maybe_unused]] const std::string &base_link = "",
                  [[maybe_unused]] const std::string &tip_link = "",
                  [[maybe_unused]] double timeout = 0.0) override;

  /**
   * @brief 求解逆运动学
   * @param target_transform 目标变换
   * @param solution 求解结果关节角度
   * @param seed_joints 种子关节角度（可选）
   * @return 求解是否成功
   */
  bool SolveIK(const tf2::Transform &target_transform,
               std::vector<double> &solution,
               [[maybe_unused]] const std::vector<double> &seed_joints = {}) override;

  /**
   * @brief 获取关节数量
   * @return 关节数量
   */
  size_t GetNumJoints() const override;

  /**
   * @brief 获取关节限制
   * @param lower_limits 下限
   * @param upper_limits 上限
   * @return 获取是否成功
   */
  bool GetJointLimits(std::vector<double> &lower_limits,
                      std::vector<double> &upper_limits) const override;

  /**
   * @brief 检查是否已初始化
   * @return 初始化状态
   */
  bool IsInitialized() const override;

  /**
   * @brief 获取运动学链信息
   * @return 关节名称列表
   */
  std::vector<std::string> GetJointNames() const override;

  /**
   * @brief 获取关节占位数映射
   * @return 关节名称到其在数组中索引位置的映射
   */
  std::map<std::string, int> GetJointPlaceholdMap() const override;

  /**
   * @brief 将关节状态对齐到IK求解器的关节顺序
   * @param joint_names 关节名称列表
   * @param joint_positions 关节位置列表
   * @param seed_joints 输出的种子关节角度（按IK求解器的关节顺序）
   * @return 对齐是否成功
   */
  bool AlignJointStateToIk(const std::vector<std::string> &joint_names,
                           const std::vector<double> &joint_positions,
                           std::vector<double> &seed_joints) const override;

  /**
   * @brief 检查目标位置是否在工作空间内
   * @param target_transform 目标变换
   * @return 是否在工作空间内
   */
  bool CheckWorkspace(const tf2::Transform &target_transform) const override;

private:
  /**
   * @brief 从tf2::Transform转换为KDL帧
   * @param transform tf2变换
   * @return KDL帧
   */
  KDL::Frame TransformToKDLFrame(const tf2::Transform &transform) const;

  /**
   * @brief 验证关节角度是否在限制范围内
   * @param joints 关节角度
   * @return 验证结果
   */
  bool ValidateJoints(const std::vector<double> &joints) const;

  /**
   * @brief 加载URDF模型
   * @param urdf_param URDF文件内容字符串
   * @return 加载是否成功
   */
  bool LoadURDF(const std::string &urdf_param);

  /**
   * @brief 设置默认关节限制（URDF加载失败时的后备方案）
   */
  void SetDefaultJointLimits();

  /**
   * @brief 打印所有机械臂长度参数
   */
  void PrintArmLengthParameters() const;

  /**
   * @brief 从URDF中解析机械臂结构参数
   * @return 解析是否成功
   */
  bool ParseKinematicsParameters();

  /**
   * @brief 解析关节限制和RPY偏移量
   * @return 解析是否成功
   */
  bool ParseJointLimitsAndOffsets();

  /**
   * @brief 解析工作空间参数（min_arm_reach_ 和 max_arm_reach_）
   */
  void ParseWorkspaceParameters();

  /**
   * @brief 解析tip link在初始姿态下的方向
   * @param base_link 基座链接名称
   * @param tip_link 末端链接名称
   * @return 解析是否成功
   */
  bool ParseTipLinkInitialOrientation(const std::string &base_link,
                                      const std::string &tip_link);

  /**
   * @brief 解析末端执行器在初始姿态下的方向
   * 使用基类中的 end_effector_frame_ 作为目标 link 名称
   * @param base_link 基座链接名称（可选，用于限定累积范围）
   * @return 解析是否成功
   */
  bool ParseEeFrameInitialOrientation(const std::string &base_link);

  /**
   * @brief 极坐标逆运动学核心算法
   * @param target_transform 目标变换
   * @param solution 求解结果关节角度
   * @return 求解是否成功
   */
  bool SolvePolarIK(const tf2::Transform &target_transform,
                    std::vector<double> &solution);

  /**
   * @brief 解析求解垂直平面2D位置 - 使用余弦定理
   * @param r 径向距离
   * @param z 垂直高度
   * @param shoulder_lift_angle 肩部抬升角度（输出）
   * @param elbow_flex_angle 肘部弯曲角度（输出）
   * @return 求解是否成功
   */
  bool SolveVerticalPlane2D(double r, double z, double &shoulder_lift_angle,
                            double &elbow_flex_angle);

  /**
   * @brief 计算末端执行器朝向角度
   * @param target_transform 目标变换
   * @param shoulder_lift_angle 肩部抬升角度
   * @param elbow_flex_angle 肘部弯曲角度
   * @return std::pair<double, double> 腕部弯曲角度和滚转角度
   */
  std::pair<double, double>
  CalculateEndEffectorOrientation(const tf2::Transform &target_transform,
                                  double shoulder_lift_angle,
                                  double elbow_flex_angle);

  // 机械臂结构参数（从URDF动态读取）
  double base_to_shoulder_pan_height_; // 基座高度（base到shoulder_pan）
  double base_to_shoulder_lift_height_; // 基座到shoulder_lift的高度
  double shoulder_pan_offset_; // shoulder_pan为旋转关节，无长度偏移
  double upper_arm_length_; // shoulder_lift到elbow_flex距离（upper_arm长度）
  double lower_arm_length_; // elbow_flex到wrist_flex距离（lower_arm长度）
  double wrist_flex_to_roll_offset_;    // wrist_flex到wrist_roll距离
  double wrist_roll_to_gripper_offset_; // wrist_roll到gripper末端距离

  // 工作空间参数
  double min_arm_reach_; // 最小工作半径
  double max_arm_reach_; // 最大工作半径

  // URDF模型
  std::unique_ptr<urdf::Model> urdf_model_;

  // 配置参数
  double position_tolerance_;
  double orientation_tolerance_;

  // 关节信息
  std::vector<std::string> joint_names_;
  std::vector<double> joint_lower_limits_;
  std::vector<double> joint_upper_limits_;
  std::vector<double>
      joint_origin_rpy_offsets_; // 每个关节的RPY偏移量（从URDF origin获取）
  std::map<std::string, int> joint_placehold_map_;
  size_t num_joints_;

  tf2::Transform tip_link_initial_transform_;
  tf2::Transform ee_link_initial_transform_;

  // 初始化状态
  bool initialized_;

  // 线程安全
  mutable std::mutex solver_mutex_;
};

} // namespace lerobot_vr_controller

#endif // LEROBOT_VR_CONTROLLER_XLEROBOT_KINEMATICS_H_
