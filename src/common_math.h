#pragma once

#include "log.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>

namespace lerobot_vr_controller {

enum class Plane { kXY, kXZ, kYZ };

enum class Axis { kX, kY, kZ };

inline double GetRoll(const tf2::Transform &transform) {
  const tf2::Quaternion q = transform.getRotation();
  if (q.length2() <= std::numeric_limits<double>::epsilon()) {
    return 0.0;
  }

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::getEulerYPR(q, yaw, pitch, roll);
  return roll;
}

inline double GetYaw(const tf2::Transform &transform) {
  const tf2::Quaternion q = transform.getRotation();
  if (q.length2() <= std::numeric_limits<double>::epsilon()) {
    return 0.0;
  }

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::getEulerYPR(q, yaw, pitch, roll);
  return yaw;
}

inline double GetPitch(const tf2::Transform &transform) {
  const tf2::Quaternion q = transform.getRotation();
  if (q.length2() <= std::numeric_limits<double>::epsilon()) {
    return 0.0;
  }

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::getEulerYPR(q, yaw, pitch, roll);
  return pitch;
}

//当指定的轴的方向与平面的法向量方向相反时，则返回负数
inline double GetIntersectionAngle(const tf2::Transform &transform, Plane plane,
                                   Axis axis) {
  const tf2::Quaternion q = transform.getRotation();

  tf2::Vector3 unit(1.0, 0.0, 0.0);
  if (axis == Axis::kY) {
    unit = tf2::Vector3(0.0, 1.0, 0.0);
  }
  if (axis == Axis::kZ) {
    unit = tf2::Vector3(0.0, 0.0, 1.0);
  }

  tf2::Vector3 n(0.0, 0.0, 1.0);
  if (plane == Plane::kXZ) {
    n = tf2::Vector3(0.0, 1.0, 0.0);
  }
  if (plane == Plane::kYZ) {
    n = tf2::Vector3(1.0, 0.0, 0.0);
  }

  if (q.length2() <= std::numeric_limits<double>::epsilon()) {
    const double dot = std::fabs(unit.dot(n));
    const double clamped = std::clamp(dot, 0.0, 1.0);
    return std::asin(clamped);
  }

  const tf2::Matrix3x3 basis = transform.getBasis();
  tf2::Vector3 dir = basis * unit;
  const double len2 = dir.length2();
  if (len2 <= std::numeric_limits<double>::epsilon()) {
    return 0.0;
  }
  dir.normalize();

  //当与平面的法向量方向相反时，则返回负数
  const double dot = dir.dot(n);
  const double clamped = std::clamp(dot, -1.0, 1.0);
  return std::asin(clamped);
}

} // namespace lerobot_vr_controller
