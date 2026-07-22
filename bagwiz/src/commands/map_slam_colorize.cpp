// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "map_slam_colorize.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/core/slam/colorize_rasterizer.hpp"
#include "map_slam_threads.hpp"  // NOLINT(build/include_subdir) src-local shared header

#ifdef BAGWIZ_WITH_SLAM_CUDA
#include "bagwiz/core/slam/colorize_rasterizer_gpu.hpp"
#endif

#include <memory>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

int colorize_thread_count(int num_threads)
{
  return resolve_threads(num_threads);
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
  bool use_gpu, std::shared_ptr<const core::slam::ColorizeGeometry> geometry,
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

    std::unique_ptr<core::slam::ColorizeRasterizer> rasterizer;
    // In non-CUDA builds use_gpu is ignored (CPU rasterizer is the only option).
    (void)use_gpu;
#ifdef BAGWIZ_WITH_SLAM_CUDA
    if (use_gpu) {
      rasterizer = core::slam::make_gpu_colorize_rasterizer(
        points, geometry ? std::span<const float>(geometry->spacings) : std::span<const float>{},
        config.rasterizer, geometry ? &geometry->tree : nullptr);
    }
#endif
    colorizers.push_back(
      std::make_unique<core::slam::MapColorizer>(
        config, geometry, points, trajectory, std::move(rasterizer)));
  }
  return colorizers;
}

}  // namespace bagwiz::commands
