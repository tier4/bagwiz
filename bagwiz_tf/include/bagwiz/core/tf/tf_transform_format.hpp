// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__TF__TF_TRANSFORM_FORMAT_HPP_
#define BAGWIZ__CORE__TF__TF_TRANSFORM_FORMAT_HPP_

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <string>
#include <vector>

// Rendering of a single resolved rigid-body transform (the result of
// tf2::BufferCore::lookupTransform), used by `bagwiz tf static calc`. Kept free
// of I/O and colour so it is pure and unit-testable without a bag; the command
// layer decides where the strings go.
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
// chain of frames running of (path.front()) -> ... -> ref (path.back()), i.e.
// the result of lookupTransform(target=path.back(), source=path.front()). The
// label line states the direction explicitly as "of=<of>  ref=<ref>": this is
// the pose of <of> expressed in <ref>, equivalent to
// `ros2 run tf2_ros tf2_echo <ref> <of>` (note the operand order — tf2_echo
// takes the reference frame first).
//
// The frame chain is rendered on its own "  chain:" line, joined with " -> "
// (e.g. "base_link -> sensor_kit_base_link -> velodyne") so intermediate frames
// are visible; the arrow there describes the tree path, not the transform's
// direction. A single-frame path (of == ref) renders as "frame -> frame", and
// an empty path as "(unknown)" on both lines. The body mirrors the --json
// hierarchy and key names (translation under `translation`; rotation under
// `rotation` as `quaternion` plus `rpy_rad` / `rpy_deg`), one value per line
// with a two-space indent per level. Monochrome (like tf2_echo); fixed
// 6-decimal precision; the trailing newline is included.
//
// `annotation` is appended to the label line right after the "of=<of>
// ref=<ref>" endpoints (e.g. "  (static)" for `tf static calc`, which resolves
// only the static tree). Callers that do not classify transforms as static vs
// dynamic pass the default empty annotation.
std::string format_transform_human(
  const geometry_msgs::msg::TransformStamped & tf, const std::vector<std::string> & path,
  const std::string & annotation = {});

// Machine-readable JSON rendering (pretty-printed, 2-space indent) carrying
// the same data as the human form. Schema:
//   {"of": str, "ref": str,
//    "translation": {"x","y","z"},
//    "rotation": {"quaternion": {"x","y","z","w"},
//                 "rpy_rad":    {"roll","pitch","yaw"},
//                 "rpy_deg":    {"roll","pitch","yaw"}}}
// Object keys are emitted in nlohmann's default (alphabetical) order. No
// trailing newline (nlohmann::json::dump does not add one).
std::string format_transform_json(
  const geometry_msgs::msg::TransformStamped & tf, const std::string & of_frame,
  const std::string & ref_frame);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__TF__TF_TRANSFORM_FORMAT_HPP_
