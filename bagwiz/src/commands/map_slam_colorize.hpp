// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__MAP_SLAM_COLORIZE_HPP_
#define COMMANDS__MAP_SLAM_COLORIZE_HPP_

#include "bagwiz/core/image/camera_info.hpp"
#include "bagwiz/core/slam/map_colorizer.hpp"
#include "bagwiz/core/slam/sensor_transform.hpp"
#include "bagwiz/core/tf/trajectory.hpp"

#include <array>
#include <memory>
#include <span>
#include <vector>

// Construction internals of the `map slam --cam` colorize pass, split out of
// map_slam.cpp so the thread-count rule and the per-camera colorizer setup
// can be unit-tested without driving a SLAM run. CLI-internal: this header
// lives with the command sources and is not installed.
namespace bagwiz::commands
{

// Effective thread count for the colorize pass: --threads resolved the same
// way as the mapping run (0 = hardware concurrency, positive values capped at
// it).
[[nodiscard]] int colorize_thread_count(int num_threads);

// The camera-independent geometry pre-pass (kd-tree, normals, spacings),
// built once and shared between every camera's MapColorizer (the kd-tree
// build is the expensive part). The neighbor count is the MapColorizerConfig
// default. `points` must outlive the returned geometry and must not be
// modified while it is in use.
[[nodiscard]] std::shared_ptr<const core::slam::ColorizeGeometry> build_shared_colorize_geometry(
  std::span<const std::array<float, 3>> points, int threads);

// One MapColorizer per camera, parallel to `camera_infos` / `t_cloud_cams`,
// all sharing the geometry pre-pass. `range_max` reuses the SLAM range crop:
// geometry farther than --max-range from any single viewpoint was never
// captured in one scan either. `points` / `trajectory` must outlive the
// returned colorizers.
//
// When `use_gpu` is true and this binary was built with BAGWIZ_WITH_SLAM_CUDA,
// a CUDA ColorizeRasterizer is injected into each MapColorizer; otherwise the
// CPU rasterizer is used.
[[nodiscard]] std::vector<std::unique_ptr<core::slam::MapColorizer>> build_camera_colorizers(
  std::span<const core::image::CameraInfo> camera_infos,
  std::span<const core::slam::SensorTransform> t_cloud_cams, double range_max, int threads,
  bool use_gpu, std::shared_ptr<const core::slam::ColorizeGeometry> geometry,
  std::span<const std::array<float, 3>> points, std::span<const core::TrajectoryPose> trajectory);

}  // namespace bagwiz::commands

#endif  // COMMANDS__MAP_SLAM_COLORIZE_HPP_
