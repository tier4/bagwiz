// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__TF_TRANSFORM_FORMAT_HPP_
#define BAGWIZ__CORE__TF_TRANSFORM_FORMAT_HPP_

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <string>

// Rendering of a single resolved rigid-body transform (the result of
// tf2::BufferCore::lookupTransform) for `bagwiz tf static`. Kept free of
// I/O and colour so it is pure and unit-testable without a bag; the
// command layer decides where the strings go.
namespace bagwiz::core
{

// Roll/pitch/yaw in radians, extracted from a quaternion via tf2's
// fixed-axis convention (tf2::Matrix3x3::getRPY).
struct RollPitchYaw
{
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
};

// Convert a geometry_msgs Quaternion to roll/pitch/yaw (radians). An
// all-zero quaternion (as produced by a default-initialised message) has
// no valid orientation, so it is treated as identity to avoid a NaN
// result from normalisation.
RollPitchYaw quaternion_to_rpy(const geometry_msgs::msg::Quaternion & q);

// Human-readable rendering of `tf` as the rigid transform from
// `from_frame` to `to_frame` — i.e. the result of
// lookupTransform(target=to_frame, source=from_frame), matching
// `ros2 run tf2_ros tf2_echo <from_frame> <to_frame>`. Shows translation,
// the rotation quaternion, and RPY in both radians and degrees. Monochrome
// (like tf2_echo); the trailing newline is included.
std::string format_transform_human(
  const geometry_msgs::msg::TransformStamped & tf, const std::string & from_frame,
  const std::string & to_frame);

// Machine-readable JSON rendering (pretty-printed, 2-space indent) carrying
// the same data as the human form. Schema:
//   {"from": str, "to": str,
//    "translation": {"x","y","z"},
//    "rotation": {"x","y","z","w"},
//    "rpy_rad": {"roll","pitch","yaw"},
//    "rpy_deg": {"roll","pitch","yaw"}}
// No trailing newline (nlohmann::json::dump does not add one).
std::string format_transform_json(
  const geometry_msgs::msg::TransformStamped & tf, const std::string & from_frame,
  const std::string & to_frame);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__TF_TRANSFORM_FORMAT_HPP_
