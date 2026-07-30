// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SLAM__SCAN_TO_WORLD_HPP_
#define BAGWIZ__CORE__SLAM__SCAN_TO_WORLD_HPP_

#include "bagwiz/core/slam/lidar_scan.hpp"
#include "bagwiz/core/tf/trajectory.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

// Rotation of an extracted LiDAR scan into the SLAM world frame for the
// colorize pass (`map slam --color`): each scan is the scene's occluder geometry
// at its own time, handed to MapColorizer::add_image as `dynamic_points` so
// map points sitting behind vehicles and pedestrians that left nothing in the
// accumulated map are rejected. GLIM-free plain data throughout, like
// point_cloud_io.
namespace bagwiz::core::slam
{

// Transform `scan`'s points into the world frame with the trajectory pose at
// the scan's stamp (interpolated by core::lookup_pose). Returns std::nullopt
// when the trajectory is empty or the scan's stamp falls outside the
// trajectory span — the same span rule the colorizer applies to its images:
// a clamped pose would place the occluders somewhere the platform never was.
[[nodiscard]] std::optional<std::vector<std::array<float, 3>>> scan_to_world_points(
  const LidarScan & scan, std::span<const core::TrajectoryPose> trajectory);

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__SCAN_TO_WORLD_HPP_
