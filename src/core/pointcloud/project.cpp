// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/project.hpp"

#include "bagwiz/core/pointcloud/point_cloud_reader.hpp"

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace bagwiz::core::pointcloud
{

namespace
{

struct RawProjectedPoint
{
  std::int32_t u = 0;
  std::int32_t v = 0;
  float depth = 0.0f;
  float scalar = 0.0f;
};

std::optional<float> compute_scalar(
  const PointCloudView & cloud,
  const tf2::Vector3 & point_camera,
  std::size_t point_index,
  ColorBy color_by)
{
  switch (color_by) {
    case ColorBy::kDistance:
      return std::sqrt(
        point_camera.x() * point_camera.x() + point_camera.y() * point_camera.y() +
        point_camera.z() * point_camera.z());
    case ColorBy::kX:
      return point_camera.x();
    case ColorBy::kY:
      return point_camera.y();
    case ColorBy::kZ:
      return point_camera.z();
    case ColorBy::kIntensity:
      return read_intensity(cloud, point_index);
  }
  return std::nullopt;
}

}  // namespace

ProjectionResult project_point_cloud(
  const PointCloudView & cloud,
  const camera::CameraInfo & camera_info,
  const tf2::Transform & cloud_to_camera,
  ColorBy color_by,
  color::ColorMapName color_map_name)
{
  if (color_by == ColorBy::kIntensity && !cloud.intensity_offset.has_value()) {
    return ProjectionResult{false, "intensity field missing", {}};
  }

  const auto color_map = color::make_color_map(color_map_name);

  std::vector<RawProjectedPoint> raw_points;
  raw_points.reserve(cloud.width * cloud.height);

  const std::size_t point_count = static_cast<std::size_t>(cloud.width) * cloud.height;
  for (std::size_t i = 0; i < point_count; ++i) {
    if (!is_valid_point(cloud, i)) {
      continue;
    }

    const auto x = read_float_field(cloud, i, *cloud.x_offset);
    const auto y = read_float_field(cloud, i, *cloud.y_offset);
    const auto z = read_float_field(cloud, i, *cloud.z_offset);
    if (!x.has_value() || !y.has_value() || !z.has_value()) {
      continue;
    }

    const tf2::Vector3 point_camera = cloud_to_camera * tf2::Vector3{x.value(), y.value(), z.value()};
    if (point_camera.z() <= 0.0f) {
      continue;
    }

    const float u_f =
      static_cast<float>(camera_info.K[0]) * point_camera.x() / point_camera.z() +
      static_cast<float>(camera_info.K[2]);
    const float v_f =
      static_cast<float>(camera_info.K[4]) * point_camera.y() / point_camera.z() +
      static_cast<float>(camera_info.K[5]);

    const auto u = static_cast<std::int32_t>(std::lroundf(u_f));
    const auto v = static_cast<std::int32_t>(std::lroundf(v_f));

    if (
      u < 0 || v < 0 || u >= static_cast<std::int32_t>(camera_info.width) ||
      v >= static_cast<std::int32_t>(camera_info.height)) {
      continue;
    }

    const auto scalar = compute_scalar(cloud, point_camera, i, color_by);
    if (!scalar.has_value()) {
      continue;
    }

    raw_points.push_back(RawProjectedPoint{u, v, static_cast<float>(point_camera.z()), scalar.value()});
  }

  float scalar_min = std::numeric_limits<float>::max();
  float scalar_max = std::numeric_limits<float>::lowest();
  for (const auto & p : raw_points) {
    scalar_min = std::min(scalar_min, p.scalar);
    scalar_max = std::max(scalar_max, p.scalar);
  }

  ProjectionResult result;
  result.ok = true;
  result.points.reserve(raw_points.size());
  for (const auto & p : raw_points) {
    const auto index = color::normalize(p.scalar, scalar_min, scalar_max);
    result.points.push_back(ProjectedPoint{p.u, p.v, p.depth, color::apply(color_map, index)});
  }

  std::sort(
    result.points.begin(), result.points.end(),
    [](const ProjectedPoint & a, const ProjectedPoint & b) { return a.depth < b.depth; });

  return result;
}

}  // namespace bagwiz::core::pointcloud
