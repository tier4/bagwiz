// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "map_slam_colorize.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "map_slam_mapping.hpp"  // NOLINT(build/include_subdir) src-local shared header: cap_threads_at_hardware_limit

#include <memory>
#include <vector>

namespace bagwiz::commands
{

int colorize_thread_count(int num_threads)
{
  const int capped = cap_threads_at_hardware_limit(num_threads);
  return capped > 0 ? capped : 4;
}

std::shared_ptr<const core::slam::ColorizeGeometry> build_shared_colorize_geometry(
  std::span<const std::array<float, 3>> points, int threads)
{
  const core::slam::MapColorizerConfig default_config;
  return std::make_shared<const core::slam::ColorizeGeometry>(
    core::slam::build_colorize_geometry(points, default_config.geometry_neighbors, threads));
}

std::vector<std::unique_ptr<core::slam::MapColorizer>> build_camera_colorizers(
  std::span<const core::image::CameraInfo> camera_infos,
  std::span<const core::slam::SensorTransform> t_cloud_cams, double range_max, int threads,
  std::shared_ptr<const core::slam::ColorizeGeometry> geometry,
  std::span<const std::array<float, 3>> points, std::span<const core::TrajectoryPose> trajectory)
{
  std::vector<std::unique_ptr<core::slam::MapColorizer>> colorizers;
  colorizers.reserve(camera_infos.size());
  for (std::size_t cam = 0; cam < camera_infos.size(); ++cam) {
    core::slam::MapColorizerConfig config;
    config.camera = camera_infos[cam];
    config.t_cloud_cam = t_cloud_cams[cam];
    // Reuse the SLAM range crop: geometry farther than --max-range from any
    // single viewpoint was never captured in one scan either.
    config.rasterizer.max_range = range_max;
    config.rasterizer.num_threads = threads;
    colorizers.push_back(
      std::make_unique<core::slam::MapColorizer>(config, geometry, points, trajectory));
  }
  return colorizers;
}

}  // namespace bagwiz::commands
