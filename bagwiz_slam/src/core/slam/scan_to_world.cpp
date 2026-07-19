// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/scan_to_world.hpp"

#include "bagwiz/core/pointcloud/cloud_transform.hpp"

#include <cstddef>
#include <vector>

namespace bagwiz::core::slam
{

std::optional<std::vector<std::array<float, 3>>> scan_to_world_points(
  const LidarScan & scan, std::span<const core::TrajectoryPose> trajectory)
{
  if (trajectory.empty()) {
    return std::nullopt;
  }
  if (
    scan.stamp_ns < trajectory.front().timestamp_ns ||
    scan.stamp_ns > trajectory.back().timestamp_ns) {
    return std::nullopt;
  }
  const auto pose = core::lookup_pose(scan.stamp_ns, trajectory);
  if (!pose) {
    return std::nullopt;
  }
  // Row-major 3x3 rotation, for transforming the colorization pass's occluder
  // scans into the world frame.
  const std::array<double, 9> r =
    pointcloud::quat_to_rotation_matrix(pose->qx, pose->qy, pose->qz, pose->qw);
  std::vector<std::array<float, 3>> world;
  world.reserve(scan.points.size());
  for (const auto & p : scan.points) {
    world.push_back(
      {static_cast<float>(r[0] * p[0] + r[1] * p[1] + r[2] * p[2] + pose->tx),
       static_cast<float>(r[3] * p[0] + r[4] * p[1] + r[5] * p[2] + pose->ty),
       static_cast<float>(r[6] * p[0] + r[7] * p[1] + r[8] * p[2] + pose->tz)});
  }
  return world;
}

}  // namespace bagwiz::core::slam
