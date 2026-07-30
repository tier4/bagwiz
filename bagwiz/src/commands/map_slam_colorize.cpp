// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "map_slam_colorize.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/core/image/packed_raster.hpp"
#include "bagwiz/core/slam/colorize_rasterizer.hpp"
#include "map_slam_threads.hpp"  // NOLINT(build/include_subdir) src-local shared header

#ifdef BAGWIZ_WITH_SLAM_CUDA
#include "bagwiz/core/slam/colorize_rasterizer_gpu.hpp"
#endif

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string_view>
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

bool colorize_one_image(
  core::slam::MapColorizer & colorizer, core::slam::ColorizeKeyframePicker * blur_picker,
  std::string_view type, std::span<const std::byte> payload, std::int64_t fallback_stamp_ns,
  std::span<const std::array<float, 3>> dynamic_points)
{
  auto decoded = core::image::to_packed_raster(type, payload);
  if (!decoded.ok()) {
    return false;
  }
  auto & raster = *decoded.raster;
  // Prefer the capture stamp; fall back to the bag record time when the
  // publisher left header.stamp unset.
  const std::int64_t stamp =
    raster.header_stamp_ns != 0 ? raster.header_stamp_ns : fallback_stamp_ns;
  if (blur_picker != nullptr) {
    // Best-of-bucket routing: the picker buffers the current gate bucket's
    // sharpest frame and hands back the one to colorize, if any.
    if (
      auto keyframe = blur_picker->offer(
        core::slam::ColorizeKeyframePicker::Frame{
          stamp, std::move(raster.bgr), raster.width, raster.height,
          std::vector<std::array<float, 3>>(dynamic_points.begin(), dynamic_points.end())})) {
      colorizer.add_image(
        keyframe->stamp_ns, keyframe->bgr, keyframe->width, keyframe->height,
        keyframe->dynamic_points);
    }
    return true;
  }
  colorizer.add_image(stamp, raster.bgr, raster.width, raster.height, dynamic_points);
  return true;
}

void colorize_flush_keyframes(
  core::slam::MapColorizer & colorizer, core::slam::ColorizeKeyframePicker * blur_picker)
{
  if (blur_picker == nullptr) {
    return;
  }
  if (auto keyframe = blur_picker->flush()) {
    colorizer.add_image(
      keyframe->stamp_ns, keyframe->bgr, keyframe->width, keyframe->height,
      keyframe->dynamic_points);
  }
}

}  // namespace bagwiz::commands
