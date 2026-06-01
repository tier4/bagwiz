// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__TRAJECTORY_HPP_
#define BAGWIZ__CORE__TRAJECTORY_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cstdint>
#include <istream>
#include <optional>
#include <ostream>
#include <span>
#include <string_view>
#include <vector>

namespace bagwiz::core
{

// One sample along a trajectory. Quaternion convention matches ROS /
// TUM: (qx, qy, qz, qw), Hamilton, normalized.
struct TrajectoryPose
{
  int64_t timestamp_ns = 0;
  double tx = 0.0;
  double ty = 0.0;
  double tz = 0.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 0.0;
};

// Outcome of parsing a TUM trajectory stream.
//
// `poses` carries every line that successfully decoded into 8 numeric
// fields. Lines that are empty, comment-only (start with `#`), or that
// fail to parse (wrong field count, non-numeric token, malformed
// timestamp) are counted in `skipped_lines`; callers decide whether to
// warn or fail. No partial poses are emitted.
struct TrajectoryReadResult
{
  std::vector<TrajectoryPose> poses;
  int64_t skipped_lines = 0;
};

// Write poses in the TUM trajectory format: one sample per line,
//
//     timestamp tx ty tz qx qy qz qw
//
// with the timestamp in seconds (9 decimal places so nanosecond
// precision is preserved). No comment header is emitted so the output
// drops straight into tools like evo without post-processing.
void write_tum(std::ostream & os, std::span<const TrajectoryPose> poses);

// Parse TUM trajectory lines from `is`. Mirrors `write_tum`: timestamps
// of the form "<sec>.<nsec>" are decoded by splitting on '.' and
// reassembling sec / nanosec as integers, so a write_tum -> read_tum
// round trip preserves the original nanosecond-precision header.stamp
// bit-exactly (a `static_cast<double>(ns) / 1e9` round trip would drift
// at year-2026 magnitudes).
TrajectoryReadResult read_tum(std::istream & is);

// Pack a TrajectoryPose into a TransformStamped with the given parent /
// child frames. Quaternion convention matches the pose's; the caller is
// responsible for ensuring (qx, qy, qz, qw) is normalised.
geometry_msgs::msg::TransformStamped pose_to_transform_stamped(
  const TrajectoryPose & pose, std::string_view frame_id, std::string_view child_frame_id);

// Compose the output trajectory pose for `traj dump`:
//
//   T_from_to = T_from_header * T_header_body * T_body_to
//
// `body_pose` is the message's pose — the tracked body expressed in its own
// `header.frame_id` (`T_header_body`). The two optional bridges come from the
// bag's TF tree:
//   * `from_header` re-expresses the result into the requested `--from` frame
//     (`lookupTransform(--from, header.frame_id)`); pass std::nullopt when
//     `--from` is omitted or equals `header.frame_id` (identity, no remap).
//   * `body_to` walks from the body frame to the requested `--to` frame
//     (`lookupTransform(body, --to)`, e.g. Odometry child_frame_id -> sensor
//     via static TF); pass std::nullopt when no tracked-side traversal applies.
//
// When both bridges are std::nullopt the input pose is returned verbatim (no
// quaternion renormalisation), so a no-transform dump round-trips bit-exactly.
// Quaternion convention matches ROS / tf2 (Hamilton).
geometry_msgs::msg::Pose compose_trajectory_pose(
  const std::optional<geometry_msgs::msg::Transform> & from_header,
  const geometry_msgs::msg::Pose & body_pose,
  const std::optional<geometry_msgs::msg::Transform> & body_to);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__TRAJECTORY_HPP_
