// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__POINTCLOUD__PROJECTOR_HPP_
#define BAGWIZ__CORE__POINTCLOUD__PROJECTOR_HPP_

#include "bagwiz/core/image/camera_info.hpp"
#include "bagwiz/core/image/packed_raster.hpp"
#include "bagwiz/core/pointcloud/pointcloud2.hpp"
#include "bagwiz/core/pointcloud/property.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace bagwiz::core::pointcloud
{

struct ProjectedPoint
{
  std::int32_t u = 0;
  std::int32_t v = 0;
  float depth = 0.0f;
  float value = 0.0f;
};

struct ProjectionResult
{
  std::vector<ProjectedPoint> points;
  std::string error;

  [[nodiscard]] bool ok() const noexcept { return error.empty(); }
};

// Per-point BGR color sampled from an image. The vector is parallel to the input
// point cloud: one entry for every input point. Points that project outside the
// image or fall behind the camera are colored black ({0, 0, 0}).
struct ColorizeResult
{
  std::vector<std::array<std::uint8_t, 3>> colors;
  std::string error;

  [[nodiscard]] bool ok() const noexcept { return error.empty(); }
};

// Project a point cloud onto an image.
// When `use_rectified` is true, the projection uses `camera_info.p` (the rectified projection
// matrix) instead of `camera_info.k`. This should be used when the target image has been
// undistorted so the projected points align with the rectified image.
[[nodiscard]] ProjectionResult project_pointcloud(
  const PointCloud2 & cloud, const image::CameraInfo & camera_info,
  const std::array<double, 16> & transform, std::uint32_t image_width, std::uint32_t image_height,
  PointCloudProperty property, bool use_rectified = false);

// Colorize a point cloud by projecting each point into `image` and sampling the
// nearest pixel. `transform` maps points from the cloud frame into the camera
// frame, using the same column-major layout as project_pointcloud(). The camera
// intrinsics are taken from `camera_info.k` (or `camera_info.p` when
// `use_rectified` is true). Points whose projection lies outside the image bounds
// or behind the camera are returned as black.
[[nodiscard]] ColorizeResult colorize_pointcloud(
  const PointCloud2 & cloud, const image::CameraInfo & camera_info,
  const image::PackedRaster & image, const std::array<double, 16> & transform,
  bool use_rectified = false);

}  // namespace bagwiz::core::pointcloud

#endif  // BAGWIZ__CORE__POINTCLOUD__PROJECTOR_HPP_
