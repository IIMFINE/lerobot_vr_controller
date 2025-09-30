#ifndef LEROBOT_VR_CONTROLLER_KINEMATICS_H_
#define LEROBOT_VR_CONTROLLER_KINEMATICS_H_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <tf2/LinearMath/Transform.h>

namespace lerobot_vr_controller {

/**
 * @brief 运动学求解器类型枚举
 */
enum class KinematicsType {
  XLEROBOT      // 使用极坐标的解析求解器
};

/**
 * @brief 运动学求解器基类
 * 
 * 定义了所有运动学求解器必须实现的通用接口
 */
class KinematicsInterface {
public:
  /**
   * @brief 虚析构函数
   */
  virtual ~KinematicsInterface() = default;

  /**
   * @brief 初始化运动学求解器
   * @param urdf_param URDF参数或内容字符串
   * @param base_link 基座链接名称（默认为空，使用默认值）
   * @param tip_link 末端链接名称（默认为空，使用默认值）
   * @param timeout 求解超时时间（秒，默认为空，使用默认值）
   * @return 初始化是否成功
   */
  virtual bool Initialize(const std::string &urdf_param,
                         const std::string &base_link = "",
                         const std::string &tip_link = "",
                         double timeout = 0.0) = 0;

  /**
   * @brief 求解逆运动学
   * @param target_transform 目标变换
   * @param solution 求解结果关节角度
   * @param seed_joints 种子关节角度（可选）
   * @return 求解是否成功
   */
  virtual bool SolveIK(const tf2::Transform &target_transform,
                      std::vector<double> &solution,
                      const std::vector<double> &seed_joints = {}) = 0;

  /**
   * @brief 获取关节数量
   * @return 关节数量
   */
  virtual size_t GetNumJoints() const = 0;

  /**
   * @brief 获取关节限制
   * @param lower_limits 下限
   * @param upper_limits 上限
   * @return 获取是否成功
   */
  virtual bool GetJointLimits(std::vector<double> &lower_limits,
                             std::vector<double> &upper_limits) const = 0;

  /**
   * @brief 检查是否已初始化
   * @return 初始化状态
   */
  virtual bool IsInitialized() const = 0;

  /**
   * @brief 获取运动学链信息
   * @return 关节名称列表
   */
  virtual std::vector<std::string> GetJointNames() const = 0;

  /**
   * @brief 获取关节占位数映射
   * @return 关节名称到其在数组中索引位置的映射
   */
  virtual std::map<std::string, int> GetJointPlaceholdMap() const = 0;

  /**
   * @brief 将关节状态对齐到IK求解器的关节顺序
   * @param joint_names 关节名称列表
   * @param joint_positions 关节位置列表
   * @param seed_joints 输出的种子关节角度（按IK求解器的关节顺序）
   * @return 对齐是否成功
   */
  virtual bool AlignJointStateToIk(const std::vector<std::string> &joint_names,
                                  const std::vector<double> &joint_positions,
                                  std::vector<double> &seed_joints) const = 0;

  /**
   * @brief 检查目标位置是否在工作空间内
   * @param target_transform 目标变换
   * @return 是否在工作空间内
   */
  virtual bool CheckWorkspace(const tf2::Transform &target_transform) const = 0;

  /**
   * @brief 设置求解容忍度（可选接口）
   * @param position_tolerance 位置容忍度
   * @param orientation_tolerance 方向容忍度
   */
  virtual void SetTolerances([[maybe_unused]] double position_tolerance,
                            [[maybe_unused]] double orientation_tolerance) {}

  /**
   * @brief 设置求解超时时间（可选接口）
   * @param timeout 超时时间（秒）
   */
  virtual void SetTimeout([[maybe_unused]] double timeout) {}

  /**
   * @brief 设置末端执行器链接名称（可选接口）
   * @param end_effector_frame 末端执行器链接名称
   */
  virtual void SetEndEffectorFrame(const std::string &end_effector_frame);

protected:
  // 末端执行器链接名称
  std::string end_effector_frame_;
};

/**
 * @brief 运动学求解器工厂类
 */
class KinematicsFactory {
public:
  /**
   * @brief 创建运动学求解器
   * @param type 求解器类型
   * @return 运动学求解器智能指针
   */
  static std::unique_ptr<KinematicsInterface> CreateKinematics(KinematicsType type);

  /**
   * @brief 根据字符串创建运动学求解器
   * @param type_str 求解器类型字符串 ("xlerobot")
   * @return 运动学求解器智能指针
   */
  static std::unique_ptr<KinematicsInterface> CreateKinematics(const std::string &type_str);

  /**
   * @brief 获取所有可用的运动学求解器类型
   * @return 类型字符串列表
   */
  static std::vector<std::string> GetAvailableTypes();

private:
  KinematicsFactory() = default;
};

} // namespace lerobot_vr_controller

#endif // LEROBOT_VR_CONTROLLER_KINEMATICS_H_
