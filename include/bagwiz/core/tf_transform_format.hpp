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
#include <vector>

// Rendering of a single resolved rigid-body transform (the result of
// tf2::BufferCore::lookupTransform), shared by `bagwiz tf static` and
// `bagwiz tf walk`. Kept free of I/O and colour so it is pure and
// unit-testable without a bag; the command layer decides where the strings go.
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

// Human-readable rendering of `tf` as the rigid transform along `path` — the
// chain of frames between the source (path.front()) and target (path.back()),
// i.e. the result of lookupTransform(target=path.back(), source=path.front()),
// matching `ros2 run tf2_ros tf2_echo <from> <to>`. The direction line shows
// the full chain joined with " -> " (e.g.
// "base_link -> sensor_kit_base_link -> velodyne") so intermediate frames are
// visible, not just the endpoints; a single-frame path (source == target)
// renders as "frame -> frame", and an empty path as "(unknown)". The body
// mirrors the --json hierarchy and key names (translation under `translation`;
// rotation under `rotation` as `quaternion` plus `rpy_rad` / `rpy_deg`), one
// value per line with a two-space indent per level. Monochrome (like
// tf2_echo); fixed 6-decimal precision; the trailing newline is included.
//
// `annotation` is appended to the direction line right after the chain (e.g.
// "  (static)" for `tf static`, which resolves only the static tree). `tf
// walk` does not classify transforms as static vs dynamic, so it passes the
// default empty annotation.
std::string format_transform_human(
  const geometry_msgs::msg::TransformStamped & tf, const std::vector<std::string> & path,
  const std::string & annotation = {});

// Machine-readable JSON rendering (pretty-printed, 2-space indent) carrying
// the same data as the human form. Schema:
//   {"from": str, "to": str,
//    "translation": {"x","y","z"},
//    "rotation": {"quaternion": {"x","y","z","w"},
//                 "rpy_rad":    {"roll","pitch","yaw"},
//                 "rpy_deg":    {"roll","pitch","yaw"}}}
// Object keys are emitted in nlohmann's default (alphabetical) order. No
// trailing newline (nlohmann::json::dump does not add one).
std::string format_transform_json(
  const geometry_msgs::msg::TransformStamped & tf, const std::string & from_frame,
  const std::string & to_frame);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__TF_TRANSFORM_FORMAT_HPP_
