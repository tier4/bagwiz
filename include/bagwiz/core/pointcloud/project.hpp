// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__POINTCLOUD__PROJECT_HPP_
#define BAGWIZ__CORE__POINTCLOUD__PROJECT_HPP_

#include "bagwiz/core/camera/camera_info.hpp"
#include "bagwiz/core/color/color_map.hpp"
#include "bagwiz/core/pointcloud/point_cloud_reader.hpp"
#include "bagwiz/core/pointcloud/types.hpp"

#include <tf2/LinearMath/Transform.h>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace bagwiz::core::pointcloud
{

// A single point projected from a point cloud into image space.
struct ProjectedPoint
{
  std::int32_t u = 0;
  std::int32_t v = 0;
  float depth = 0.0f;
  color::Rgb rgb;
};

// Outcome of project_point_cloud(). On success `points` holds the projected,
// colored, depth-sorted pixels and `error` is empty. On failure `points` is
// empty and `error` carries a human-readable reason.
struct ProjectionResult
{
  std::optional<std::vector<ProjectedPoint>> points;
  std::string error;

  [[nodiscard]] bool ok() const noexcept { return points.has_value() && error.empty(); }
};

// Project a point cloud into a camera image, color each projected point by the
// requested scalar, and sort the result front-to-back by depth.
//
// The projection uses camera_info.K directly (fx, fy, cx, cy) and ignores
// distortion coefficients (D), rectification (R), and the projection matrix
// (P). Callers are expected to supply a camera_info that has been pre-rectified
// if necessary.
ProjectionResult project_point_cloud(
  const PointCloudView & cloud, const camera::CameraInfo & camera_info,
  const tf2::Transform & cloud_to_camera, ColorBy color_by, color::ColorMapName color_map_name);

}  // namespace bagwiz::core::pointcloud

#endif  // BAGWIZ__CORE__POINTCLOUD__PROJECT_HPP_
