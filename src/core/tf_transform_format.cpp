// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tf_transform_format.hpp"

#include <nlohmann/json.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <iomanip>
#include <numbers>
#include <sstream>
#include <string>

namespace bagwiz::core
{

namespace
{

constexpr double kRadToDeg = 180.0 / std::numbers::pi;
// Below this squared length a quaternion carries no usable orientation
// (an all-zero default-initialised message lands here).
constexpr double kMinQuatLength2 = 1e-12;
// Fixed decimal places for the human form (matches tf2_echo).
constexpr int kHumanDecimals = 6;
// Indent width for the pretty-printed --json output.
constexpr int kJsonIndent = 2;

}  // namespace

RollPitchYaw quaternion_to_rpy(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);

  // tf2::Matrix3x3 builds its rotation matrix from the raw components
  // without normalising, so a non-unit quaternion (a zero message, or one
  // with floating-point drift that slips past tf2's own setTransform check)
  // skews the matrix and yields meaningless RPY. Normalise first; a
  // ~zero-length quaternion has no valid orientation, so fall back to
  // identity instead of dividing by zero.
  if (tf_q.length2() < kMinQuatLength2) {
    return {0.0, 0.0, 0.0};
  }
  tf_q.normalize();

  const tf2::Matrix3x3 m(tf_q);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  m.getRPY(roll, pitch, yaw);
  return {roll, pitch, yaw};
}

std::string format_transform_human(
  const geometry_msgs::msg::TransformStamped & tf, const std::string & from_frame,
  const std::string & to_frame)
{
  const auto & t = tf.transform.translation;
  const auto & r = tf.transform.rotation;
  const RollPitchYaw rpy = quaternion_to_rpy(r);

  // std::ostringstream rather than fmt: bagwiz_core is built with
  // FMT_HEADER_ONLY, and fmt's consteval format-string checking trips the
  // clang-tidy pass when instantiated here (the same reason renderer.cpp
  // avoids fmt formatting). Fixed 6-decimal precision matches tf2_echo.
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(kHumanDecimals);
  oss << "Transform: " << from_frame << " -> " << to_frame << "  (static)\n";
  oss << "  Translation (x, y, z):          [" << t.x << ", " << t.y << ", " << t.z << "]\n";
  oss << "  Rotation quaternion (x,y,z,w):  [" << r.x << ", " << r.y << ", " << r.z << ", " << r.w
      << "]\n";
  oss << "  Rotation RPY (rad):             [" << rpy.roll << ", " << rpy.pitch << ", " << rpy.yaw
      << "]\n";
  oss << "  Rotation RPY (deg):             [" << rpy.roll * kRadToDeg << ", "
      << rpy.pitch * kRadToDeg << ", " << rpy.yaw * kRadToDeg << "]\n";
  return oss.str();
}

std::string format_transform_json(
  const geometry_msgs::msg::TransformStamped & tf, const std::string & from_frame,
  const std::string & to_frame)
{
  const auto & t = tf.transform.translation;
  const auto & r = tf.transform.rotation;
  const RollPitchYaw rpy = quaternion_to_rpy(r);

  nlohmann::json j;
  j["from"] = from_frame;
  j["to"] = to_frame;
  j["translation"] = {{"x", t.x}, {"y", t.y}, {"z", t.z}};
  j["rotation"] = {{"x", r.x}, {"y", r.y}, {"z", r.z}, {"w", r.w}};
  j["rpy_rad"] = {{"roll", rpy.roll}, {"pitch", rpy.pitch}, {"yaw", rpy.yaw}};
  j["rpy_deg"] = {
    {"roll", rpy.roll * kRadToDeg}, {"pitch", rpy.pitch * kRadToDeg}, {"yaw", rpy.yaw * kRadToDeg}};

  return j.dump(kJsonIndent);
}

}  // namespace bagwiz::core
