#ifndef LEROBOT_VR_CONTROLLER_KINEMATICS_H_
#define LEROBOT_VR_CONTROLLER_KINEMATICS_H_

#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <tf2/LinearMath/Transform.h>
#include <trac_ik/trac_ik.hpp>
#include <urdf/model.h>

namespace lerobot_vr_controller {

/**
 * @brief SoArm101机器人逆运动学求解器
 *
 * 使用TRAC-IK库进行高效的逆运动学求解，支持多解和约束处理
 */
class SoArm101Kinematics {
public:
  /**
   * @brief 构造函数
   */
  SoArm101Kinematics();

  /**
   * @brief 析构函数
   */
  ~SoArm101Kinematics() = default;

  // 禁用拷贝构造和赋值操作
  SoArm101Kinematics(const SoArm101Kinematics &) = delete;
  SoArm101Kinematics &operator=(const SoArm101Kinematics &) = delete;

  /**
   * @brief 初始化运动学求解器
   * @param urdf_param URDF参数名称
   * @param base_link 基座链接名称
   * @param tip_link 末端链接名称
   * @param timeout 求解超时时间（秒）
   * @return 初始化是否成功
   */
  bool Initialize(const std::string &urdf_param = "robot_description",
                  const std::string &base_link = "base_link",
                  const std::string &tip_link = "gripper_link",
                  double timeout = 0.005);

  /**
   * @brief 求解逆运动学
   * @param target_transform 目标变换
   * @param solution 求解结果关节角度
   * @param seed_joints 种子关节角度（可选）
   * @return 求解是否成功
   */
  bool SolveIK(const tf2::Transform &target_transform,
               std::vector<double> &solution,
               const std::vector<double> &seed_joints = {});

  /**
   * @brief 求解逆运动学（使用KDL数据类型）
   * @param target_frame 目标坐标系
   * @param seed_joints 种子关节角度
   * @param solution 求解结果关节角度
   * @return 求解是否成功
   */
  bool SolveIK(const KDL::Frame &target_frame, const KDL::JntArray &seed_joints,
               KDL::JntArray &solution);

  /**
   * @brief 获取关节数量
   * @return 关节数量
   */
  size_t GetNumJoints() const;

  /**
   * @brief 获取关节限制
   * @param lower_limits 下限
   * @param upper_limits 上限
   * @return 获取是否成功
   */
  bool GetJointLimits(std::vector<double> &lower_limits,
                      std::vector<double> &upper_limits) const;

  /**
   * @brief 设置求解超时时间
   * @param timeout 超时时间（秒）
   */
  void SetTimeout(double timeout);

  /**
   * @brief 设置IK求解容忍度
   * @param position_tolerance 位置容忍度（米）
   * @param orientation_tolerance 方向容忍度（弧度）
   */
  void SetTolerances(double position_tolerance, double orientation_tolerance);

  /**
   * @brief 检查是否已初始化
   * @return 初始化状态
   */
  bool IsInitialized() const;

  /**
   * @brief 获取运动学链信息
   * @return 关节名称列表
   */
  std::vector<std::string> GetJointNames() const;

  /**
   * @brief 获取关节占位数映射
   * @return 关节名称到其在数组中索引位置的映射
   */
  std::map<std::string, int> GetJointPlaceholdMap() const;

  /**
   * @brief 将关节状态对齐到IK求解器的关节顺序
   * @param joint_names 关节名称列表
   * @param joint_positions 关节位置列表
   * @param seed_joints 输出的种子关节角度（按IK求解器的关节顺序）
   * @return 对齐是否成功
   */
  bool AlignJointStateToIk(const std::vector<std::string> &joint_names,
                           const std::vector<double> &joint_positions,
                           std::vector<double> &seed_joints) const;

  /**
   * @brief 检查目标位置是否在工作空间内
   * @param target_transform 目标变换
   * @return 是否在工作空间内
   */
  bool CheckWorkspace(const tf2::Transform &target_transform) const;

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
   * @param urdf_string URDF参数名称
   * @return 加载是否成功
   */
  bool LoadURDF(const std::string &urdf_string);

  // 智能种子生成
  void GenerateSmartSeeds(const tf2::Transform &target_transform,
                          const std::vector<double> &current_joints,
                          std::vector<std::vector<double>> &smart_seeds) const;
  void GeneratePredefinedSeeds();

  // TRAC-IK求解器
  std::unique_ptr<TRAC_IK::TRAC_IK> trac_ik_solver_;

  // 运动学链和模型
  std::unique_ptr<urdf::Model> urdf_model_;
  KDL::Tree kdl_tree_;
  KDL::Chain kinematic_chain_;

  // 配置参数
  std::string base_link_;
  std::string tip_link_;
  double timeout_;
  double position_tolerance_;
  double orientation_tolerance_;

  // 关节信息
  std::vector<std::string> joint_names_;
  std::vector<double> joint_lower_limits_;
  std::vector<double> joint_upper_limits_;
  std::map<std::string, int> joint_placehold_map_;
  size_t num_joints_;

  // 初始化状态
  bool initialized_;

  // 缓存和优化相关
  mutable std::vector<double> last_solution_cache_;
  mutable std::mutex solution_cache_mutex_;

  // 智能种子生成
  std::vector<std::vector<double>> predefined_seeds_;
};

} // namespace lerobot_vr_controller

#endif // LEROBOT_VR_CONTROLLER_KINEMATICS_H_
