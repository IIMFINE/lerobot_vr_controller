#include "joint_position_filter.h"

#include <cmath>
#include <stdexcept>

#include "log.h"

JointPositionFilter::JointPositionFilter(double alpha, double initial_value)
    : alpha_(alpha), filtered_value_(initial_value), first_run_(true) {
  if (alpha_ <= 0.0 || alpha_ > 1.0) {
    throw std::out_of_range("Alpha must be in range (0, 1]");
  }
}

double JointPositionFilter::Filter(double measurement) {
  if (first_run_) {
    // 第一次运行时直接使用测量值作为初始滤波值
    filtered_value_ = measurement;
    first_run_ = false;
    LE_LOG_INFO << "Joint position filter initialized with value: "
                << measurement << std::endl;
  } else {
    // 检查角度变化是否超过10度
    double angle_diff_rad = measurement - filtered_value_;
    double angle_diff_deg = std::abs(angle_diff_rad) * 180.0 / M_PI;
    if (angle_diff_deg > 10.0) {
      // 角度变化超过10度，根据变化方向在filtered_value_基础上加减10度
      double ten_degrees_rad = 10.0 * M_PI / 180.0;
      if (angle_diff_rad > 0) {
        filtered_value_ += ten_degrees_rad;
      } else {
        filtered_value_ -= ten_degrees_rad;
      }
      LE_LOG_INFO_T(1s) << "Angle change exceeds 10 degrees (diff: "
                        << angle_diff_deg
                        << "), adjusted filtered value to: " << filtered_value_
                        << std::endl;
      return filtered_value_;
    }
    // 应用一阶低通滤波公式
    filtered_value_ = alpha_ * measurement + (1.0 - alpha_) * filtered_value_;
  }
  return filtered_value_;
}

void JointPositionFilter::Reset(double value) {
  filtered_value_ = value;
  first_run_ = true;
}

double JointPositionFilter::GetValue() const { return filtered_value_; }

void JointPositionFilter::SetAlpha(double alpha) {
  if (alpha <= 0.0 || alpha > 1.0) {
    throw std::out_of_range("Alpha must be in range (0, 1]");
  }
  alpha_ = alpha;
}

double JointPositionFilter::GetAlpha() const { return alpha_; }
