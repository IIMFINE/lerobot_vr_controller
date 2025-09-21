#include "joint_position_filter.h"

#include <stdexcept>

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
  } else {
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
