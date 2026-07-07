// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__POINTCLOUD__DESKEW_HPP_
#define BAGWIZ__CORE__POINTCLOUD__DESKEW_HPP_

#include "bagwiz/core/pointcloud/pointcloud2.hpp"
#include "bagwiz/core/trajectory.hpp"

#include <geometry_msgs/msg/transform.hpp>

#include <cstdint>
#include <optional>
#include <span>
#include <string>

namespace bagwiz::core::pointcloud
{

// Outcome of deskew_pointcloud2(). On success `cloud` holds the deskewed
// points and `error` is empty; on failure `cloud` is reset and `error` carries
// the reason. The counters classify every point of the input (they sum to
// `points_total` except when the whole cloud is rejected up front):
//   - points_deskewed:  moved to the reference pose.
//   - points_no_time:   no usable per-point time (cloud-wide, or a single
//                        point whose time value is non-finite).
//   - points_no_pose:   no trajectory pose could be resolved for the point's
//                        (or the reference's) timestamp.
//   - points_nonfinite: xyz itself is NaN/Inf; passed through unchanged.
struct DeskewResult
{
  std::optional<PointCloud2> cloud;
  std::string error;
  std::uint64_t points_total = 0;
  std::uint64_t points_deskewed = 0;
  std::uint64_t points_no_time = 0;
  std::uint64_t points_no_pose = 0;
  std::uint64_t points_nonfinite = 0;

  [[nodiscard]] bool ok() const noexcept { return cloud.has_value() && error.empty(); }
};

// Deskew `input`, moving each point to the reference timestamp `t_ref_ns` using
// the world trajectory (T_world_sensor over time). `extrinsic` E maps a point
// from the cloud frame into the trajectory (`--to`) frame; nullopt = identity.
//   p' = E^{-1} * (T(t_ref)^{-1} * T(t_i)) * E * p
// Non-target fields/bytes are preserved; only xyz + one per-point time field are
// rewritten (time -> t_ref-equivalent to block downstream double-deskew). Time
// field detected per point_time.hpp. Big-endian input is rejected. A cloud with
// no usable time field is returned verbatim with points_no_time set (the command
// treats that as fatal upfront).
DeskewResult deskew_pointcloud2(
  const PointCloud2 & input, std::int64_t t_ref_ns,
  std::span<const core::TrajectoryPose> trajectory,
  const std::optional<geometry_msgs::msg::Transform> & extrinsic = std::nullopt);

}  // namespace bagwiz::core::pointcloud

#endif  // BAGWIZ__CORE__POINTCLOUD__DESKEW_HPP_
