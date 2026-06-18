// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SLAM__CLOUD_MAPPER_HPP_
#define BAGWIZ__CORE__SLAM__CLOUD_MAPPER_HPP_

#include "bagwiz/core/slam/lidar_scan.hpp"
#include "bagwiz/core/trajectory.hpp"

#include <array>
#include <memory>
#include <vector>

// LiDAR-only optimized mapping over a sequence of scans. Extends the M1
// odometry path (GLIM's OdometryEstimationCT) by routing the marginalized
// frames through GLIM's SubMapping -> GlobalMapping — the same pipeline
// glim_rosbag uses for its final globally-optimized output — and then reading
// back the optimized global point-cloud map plus the globally-optimized
// per-scan trajectory.
//
// As with CloudOdometry, every GLIM / Eigen / GTSAM type is hidden behind a
// pimpl so this header (and the `slam` command that drives it) stays free of
// GLIM includes; only cloud_mapper.cpp pulls GLIM in, and the whole translation
// unit is compiled only when BAGWIZ_WITH_SLAM is on. No ROS node / pub-sub is
// involved — GLIM's modules are called directly.
//
// Usage: feed scans in timestamp order with insert(), then call finish() once
// to flush, run the global optimization, and obtain the map + trajectory.
namespace bagwiz::core::slam
{

// Density control for CloudMapper's exported map. This is deliberately
// decoupled from GLIM's internal sub-map density: the optimization always runs
// with GLIM's stock defaults (so the trajectory is reproducible and unaffected),
// while the exported map is rebuilt from every frame's full points placed at
// their globally-optimized poses and merged at `map_resolution`. Changing this
// therefore changes only the map's appearance, never the trajectory.
struct CloudMapperConfig
{
  // Voxel side length [m] of the exported map. The optimized per-frame points are
  // merged into voxels of this size; smaller = denser. The cloud preprocessor's
  // input voxel (~0.15 m) and 1–100 m range crop still bound how fine the real
  // data is, but a value below ~0.15 m can still recover detail from the offset
  // grids of overlapping frames (at the cost of a much larger map). Must be > 0.
  double map_resolution = 0.2;
};

// Result of CloudMapper::finish(). All fields are GLIM-free plain data so the
// caller can hand them straight to write_ply / write_tum.
struct CloudMap
{
  // Globally-optimized LiDAR poses in the world frame, sorted by timestamp.
  std::vector<core::TrajectoryPose> trajectory;

  // Optimized global map: world-frame xyz, one entry per point.
  std::vector<std::array<float, 3>> points;

  // Per-point intensity, parallel to `points`. Empty unless every submap
  // carried intensities (mirrors GLIM's all-or-nothing export).
  std::vector<float> intensities;
};

class CloudMapper
{
public:
  // `config` tunes the exported map's density (see CloudMapperConfig); the
  // default reproduces GLIM's stock pipeline.
  explicit CloudMapper(CloudMapperConfig config = {});
  ~CloudMapper();

  CloudMapper(const CloudMapper &) = delete;
  CloudMapper & operator=(const CloudMapper &) = delete;
  CloudMapper(CloudMapper &&) noexcept;
  CloudMapper & operator=(CloudMapper &&) noexcept;

  // Feed one scan. Scans must arrive in non-decreasing timestamp order. A scan
  // with no per-point time is fed with explicit zero per-point times (treated
  // as already motion-undistorted), bypassing GLIM's pseudo-time synthesis.
  void insert(const LidarScan & scan);

  // Flush the remaining in-flight frames, run the global optimization, and
  // return the optimized map + trajectory. Heavy: the global matching-based
  // iSAM2 optimization runs here.
  [[nodiscard]] CloudMap finish();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__CLOUD_MAPPER_HPP_
