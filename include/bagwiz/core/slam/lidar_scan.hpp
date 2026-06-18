// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SLAM__LIDAR_SCAN_HPP_
#define BAGWIZ__CORE__SLAM__LIDAR_SCAN_HPP_

#include "bagwiz/core/pointcloud/pointcloud2.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

// SLAM-facing extraction layer that turns a parsed sensor_msgs/msg/PointCloud2
// (core::pointcloud::PointCloud2 — the byte-level parse) into a GLIM-free
// plain-data scan. Keeping the output free of GLIM / Eigen types confines the
// GLIM dependency to the odometry wrapper (built only when BAGWIZ_WITH_SLAM is
// on), so this layer and its tests compile in every build.
//
// Per-point time handling mirrors glim's `extract_raw_points` so the values are
// already in the convention glim::TimeKeeper expects:
//   - time field detected by name: "t" / "time" / "time_stamp" / "timestamp"
//   - UINT32  -> value / 1e9 (nanoseconds to seconds)
//   - FLOAT32 / FLOAT64 -> taken as-is (seconds)
// Whether those seconds are relative (to header.stamp / the first point) or
// absolute is NOT decided here; glim::TimeKeeper auto-resolves it downstream.
// When the cloud carries no usable time field, `times` is left empty and
// `has_per_point_time` is false — the caller then treats the cloud as already
// motion-undistorted (all points captured simultaneously).
namespace bagwiz::core::slam
{

// One extracted LiDAR scan. `intensities` and `times` are either empty or
// exactly `points.size()` long.
struct LidarScan
{
  std::int64_t stamp_ns = 0;  // header.stamp as nanoseconds since epoch
  std::string frame_id;       // header.frame_id (the cloud's reference frame)

  std::vector<std::array<double, 3>> points;  // xyz, one per point
  std::vector<double> intensities;            // empty when no intensity field
  std::vector<double> times;                  // seconds; empty when no time field
  bool has_per_point_time = false;            // false => already-undistorted
};

// Outcome of to_lidar_scan(). On success `scan` holds the data and `error` is
// empty; on failure `scan` is reset and `error` carries the reason.
struct LidarScanResult
{
  std::optional<LidarScan> scan;
  std::string error;

  [[nodiscard]] bool ok() const noexcept { return scan.has_value() && error.empty(); }
};

// Name of the intensity field to look for (PointCloud2 has no canonical name).
inline constexpr const char * kDefaultIntensityField = "intensity";

// Extract a LidarScan from a parsed PointCloud2. Requires `x`/`y`/`z` fields of
// FLOAT32 or FLOAT64. Intensity (named `intensity_field`) and a per-point time
// field are optional. Big-endian point data (cloud.is_bigendian) is rejected.
[[nodiscard]] LidarScanResult to_lidar_scan(
  const core::pointcloud::PointCloud2 & cloud,
  const std::string & intensity_field = kDefaultIntensityField);

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__LIDAR_SCAN_HPP_
