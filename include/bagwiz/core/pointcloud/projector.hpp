// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__POINTCLOUD__PROJECTOR_HPP_
#define BAGWIZ__CORE__POINTCLOUD__PROJECTOR_HPP_

#include "bagwiz/commands/generate_video.hpp"
#include "bagwiz/core/image/camera_info.hpp"
#include "bagwiz/core/pointcloud/pointcloud2.hpp"

#include <array>
#include <cstdint>
#include <optional>
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

[[nodiscard]] ProjectionResult project_pointcloud(
  const PointCloud2 & cloud, const image::CameraInfo & camera_info,
  const std::array<double, 16> & transform, std::uint32_t image_width, std::uint32_t image_height,
  commands::PointCloudProperty property);

}  // namespace bagwiz::core::pointcloud

#endif  // BAGWIZ__CORE__POINTCLOUD__PROJECTOR_HPP_
