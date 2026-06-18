// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/projector.hpp"

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>

namespace bagwiz::core::pointcloud
{

namespace
{

float read_field(
  const PointCloud2 & cloud, std::uint32_t point_idx, std::uint32_t offset, PointFieldType type)
{
  const std::byte * base = cloud.data.data() + point_idx * cloud.point_step + offset;
  switch (type) {
    case PointFieldType::kFloat32:
      return *reinterpret_cast<const float *>(base);
    case PointFieldType::kFloat64:
      return static_cast<float>(*reinterpret_cast<const double *>(base));
    case PointFieldType::kInt8:
      return static_cast<float>(*reinterpret_cast<const std::int8_t *>(base));
    case PointFieldType::kUint8:
      return static_cast<float>(*reinterpret_cast<const std::uint8_t *>(base));
    case PointFieldType::kInt16:
      return static_cast<float>(*reinterpret_cast<const std::int16_t *>(base));
    case PointFieldType::kUint16:
      return static_cast<float>(*reinterpret_cast<const std::uint16_t *>(base));
    case PointFieldType::kInt32:
      return static_cast<float>(*reinterpret_cast<const std::int32_t *>(base));
    case PointFieldType::kUint32:
      return static_cast<float>(*reinterpret_cast<const std::uint32_t *>(base));
  }
  return 0.0f;
}

}  // namespace

ProjectionResult project_pointcloud(
  const PointCloud2 & cloud, const image::CameraInfo & camera_info,
  const std::array<double, 16> & transform, std::uint32_t image_width, std::uint32_t image_height,
  PointCloudProperty property, bool use_rectified)
{
  ProjectionResult result;

  const auto off_x = cloud.field_offset("x");
  const auto off_y = cloud.field_offset("y");
  const auto off_z = cloud.field_offset("z");
  if (!off_x || !off_y || !off_z) {
    result.error = "point cloud is missing required x/y/z fields";
    return result;
  }

  const auto find_field = [&](const std::string & name) -> const PointField * {
    for (const auto & f : cloud.fields) {
      if (f.name == name) {
        return &f;
      }
    }
    return nullptr;
  };

  const PointField * field_x = find_field("x");
  const PointField * field_y = find_field("y");
  const PointField * field_z = find_field("z");

  const bool need_intensity = (property == PointCloudProperty::kIntensity);
  std::optional<std::uint32_t> off_intensity;
  const PointField * field_intensity = nullptr;
  if (need_intensity) {
    off_intensity = cloud.field_offset("intensity");
    if (!off_intensity) {
      result.error = "point cloud has no intensity field";
      return result;
    }
    field_intensity = find_field("intensity");
  }

  const double fx = use_rectified ? camera_info.p[0] : camera_info.k[0];
  const double fy = use_rectified ? camera_info.p[5] : camera_info.k[4];
  const double cx = use_rectified ? camera_info.p[2] : camera_info.k[2];
  const double cy = use_rectified ? camera_info.p[6] : camera_info.k[5];

  const std::uint32_t n = cloud.height * cloud.width;
  result.points.reserve(n / 4);  // rough estimate

  for (std::uint32_t i = 0; i < n; ++i) {
    const float px = read_field(cloud, i, *off_x, field_x->datatype);
    const float py = read_field(cloud, i, *off_y, field_y->datatype);
    const float pz = read_field(cloud, i, *off_z, field_z->datatype);

    // Apply 4x4 transform: assume column-major storage matching tf2::StampedTransform.
    const double tx = transform[0] * px + transform[4] * py + transform[8] * pz + transform[12];
    const double ty = transform[1] * px + transform[5] * py + transform[9] * pz + transform[13];
    const double tz = transform[2] * px + transform[6] * py + transform[10] * pz + transform[14];

    if (tz <= 0.0) {
      continue;
    }

    const double u = fx * tx / tz + cx;
    const double v = fy * ty / tz + cy;
    if (u < 0.0 || u >= image_width || v < 0.0 || v >= image_height) {
      continue;
    }

    float value = 0.0f;
    switch (property) {
      case PointCloudProperty::kX:
        value = px;
        break;
      case PointCloudProperty::kY:
        value = py;
        break;
      case PointCloudProperty::kZ:
        value = pz;
        break;
      case PointCloudProperty::kDistance:
        value = std::sqrt(px * px + py * py + pz * pz);
        break;
      case PointCloudProperty::kIntensity:
        value = read_field(cloud, i, *off_intensity, field_intensity->datatype);
        break;
    }

    ProjectedPoint pp;
    pp.u = static_cast<std::int32_t>(u);
    pp.v = static_cast<std::int32_t>(v);
    pp.depth = static_cast<float>(tz);
    pp.value = value;
    result.points.push_back(pp);
  }

  return result;
}

}  // namespace bagwiz::core::pointcloud
