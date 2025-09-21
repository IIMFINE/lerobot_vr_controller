#ifndef LEROBOT_VR_CONTROLLER_SRC_JOINT_POSITION_FILTER_H_
#define LEROBOT_VR_CONTROLLER_SRC_JOINT_POSITION_FILTER_H_

// 一阶低通滤波器类，用于关节位置滤波
//
// 实现简单的指数移动平均滤波器：
// filtered_value = α * current_measurement + (1 - α) * previous_filtered_value
//
// α 值越大，响应越快但滤波效果越差
// α 值越小，滤波效果越好但响应越慢
class JointPositionFilter {
public:
  // 构造函数
  // alpha: 平滑因子 (0.0 < alpha <= 1.0)
  // initial_value: 初始位置值
  explicit JointPositionFilter(double alpha = 0.5, double initial_value = 0.0);

  // 对新的测量值进行滤波
  // measurement: 新的测量位置
  // 返回滤波后的位置
  double Filter(double measurement);

  // 重置滤波器状态
  // value: 新的初始位置
  void Reset(double value);

  // 获取当前滤波后的值
  // 返回当前滤波后的值
  double GetValue() const;

  // 设置新的平滑因子
  // alpha: 新的平滑因子 (0.0 < alpha <= 1.0)
  void SetAlpha(double alpha);

  // 获取当前平滑因子
  // 返回当前平滑因子
  double GetAlpha() const;

private:
  double alpha_;          // 平滑因子
  double filtered_value_; // 当前滤波后的值
  bool first_run_;        // 是否第一次运行标志
};

#endif // LEROBOT_VR_CONTROLLER_SRC_JOINT_POSITION_FILTER_H_
