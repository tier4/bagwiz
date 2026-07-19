// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tf/tf_transform_format.hpp"

#include <nlohmann/json.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <cstddef>
#include <iomanip>
#include <numbers>
#include <sstream>
#include <string>
#include <vector>

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

// Render the frame chain as the human direction line. `path` is ordered
// front=source, back=target, so the frames join left-to-right with " -> "
// (e.g. "base_link -> sensor_kit_base_link -> velodyne"). A single-frame path
// (source == target) keeps the arrow form as "frame -> frame"; an empty path
// (no chain resolved) yields "(unknown)".
std::string format_chain(const std::vector<std::string> & path)
{
  if (path.empty()) {
    return "(unknown)";
  }
  if (path.size() == 1) {
    return path.front() + " -> " + path.front();
  }
  std::string out = path.front();
  for (std::size_t i = 1; i < path.size(); ++i) {
    out += " -> ";
    out += path[i];
  }
  return out;
}

// The label for the transform's direction: "of=<of>  ref=<ref>", where the
// chain runs of -> ... -> ref, so of is path.front() and ref is path.back().
// A single-frame path (of == ref, the identity) names the same frame twice; an
// empty path (no chain resolved) has no endpoints and yields "(unknown)".
std::string format_endpoints(const std::vector<std::string> & path)
{
  if (path.empty()) {
    return "(unknown)";
  }
  return "of=" + path.front() + "  ref=" + path.back();
}

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
  const geometry_msgs::msg::TransformStamped & tf, const std::vector<std::string> & path,
  const std::string & annotation)
{
  const auto & t = tf.transform.translation;
  const auto & rot = tf.transform.rotation;
  const RollPitchYaw rpy = quaternion_to_rpy(rot);

  // std::ostringstream rather than fmt: bagwiz_tf carries no fmt
  // dependency — the same reason renderer.cpp avoids fmt formatting. Fixed
  // 6-decimal precision matches tf2_echo.
  //
  // The label states the direction as "of=<of>  ref=<ref>"; the frame chain
  // moves to its own `chain:` line, where the " -> " arrow describes the tree
  // path rather than the transform's direction. The body mirrors the --json
  // hierarchy and key names: translation under `translation`; rotation under
  // `rotation` as `quaternion` plus `rpy_rad` / `rpy_deg`. One value per line,
  // two-space indent per level (YAML-like).
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(kHumanDecimals);
  oss << "transform: " << format_endpoints(path) << annotation << "\n";
  oss << "  chain: " << format_chain(path) << "\n";
  oss << "  translation:\n";
  oss << "    x: " << t.x << "\n";
  oss << "    y: " << t.y << "\n";
  oss << "    z: " << t.z << "\n";
  oss << "  rotation:\n";
  oss << "    quaternion:\n";
  oss << "      x: " << rot.x << "\n";
  oss << "      y: " << rot.y << "\n";
  oss << "      z: " << rot.z << "\n";
  oss << "      w: " << rot.w << "\n";
  oss << "    rpy_rad:\n";
  oss << "      roll: " << rpy.roll << "\n";
  oss << "      pitch: " << rpy.pitch << "\n";
  oss << "      yaw: " << rpy.yaw << "\n";
  oss << "    rpy_deg:\n";
  oss << "      roll: " << rpy.roll * kRadToDeg << "\n";
  oss << "      pitch: " << rpy.pitch * kRadToDeg << "\n";
  oss << "      yaw: " << rpy.yaw * kRadToDeg << "\n";
  return oss.str();
}

std::string format_transform_json(
  const geometry_msgs::msg::TransformStamped & tf, const std::string & of_frame,
  const std::string & ref_frame)
{
  const auto & t = tf.transform.translation;
  const auto & rot = tf.transform.rotation;
  const RollPitchYaw rpy = quaternion_to_rpy(rot);

  nlohmann::json j;
  j["of"] = of_frame;
  j["ref"] = ref_frame;
  j["translation"] = {{"x", t.x}, {"y", t.y}, {"z", t.z}};
  j["rotation"]["quaternion"] = {{"x", rot.x}, {"y", rot.y}, {"z", rot.z}, {"w", rot.w}};
  j["rotation"]["rpy_rad"] = {{"roll", rpy.roll}, {"pitch", rpy.pitch}, {"yaw", rpy.yaw}};
  j["rotation"]["rpy_deg"] = {
    {"roll", rpy.roll * kRadToDeg}, {"pitch", rpy.pitch * kRadToDeg}, {"yaw", rpy.yaw * kRadToDeg}};

  return j.dump(kJsonIndent);
}

}  // namespace bagwiz::core
