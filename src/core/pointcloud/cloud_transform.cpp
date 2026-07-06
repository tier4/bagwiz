// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/cloud_transform.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>

namespace bagwiz::core::pointcloud
{

namespace
{

// Read / write a FLOAT32 or FLOAT64 scalar at `ptr` (little-endian == host on
// every platform bagwiz targets; big-endian point data is rejected upstream).
template <typename T>
double load(const std::byte * ptr)
{
  T v{};
  std::memcpy(&v, ptr, sizeof(T));
  return static_cast<double>(v);
}
template <typename T>
void store(std::byte * ptr, double value)
{
  T v = static_cast<T>(value);
  std::memcpy(ptr, &v, sizeof(T));
}

struct XyzLayout
{
  std::uint32_t x_off = 0;
  std::uint32_t y_off = 0;
  std::uint32_t z_off = 0;
  PointFieldType type = PointFieldType::kFloat32;
};

std::optional<PointFieldType> field_type(const PointCloud2 & cloud, const std::string & name)
{
  for (const auto & f : cloud.fields) {
    if (f.name == name) {
      if (f.count != 1) {
        return std::nullopt;
      }
      return f.datatype;
    }
  }
  return std::nullopt;
}

}  // namespace

bool RigidTransform::is_identity() const
{
  const std::array<double, 9> id{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  for (std::size_t i = 0; i < 9; ++i) {
    if (rotation[i] != id[i]) {
      return false;
    }
  }
  return translation[0] == 0.0 && translation[1] == 0.0 && translation[2] == 0.0;
}

CloudTransformResult transform_cloud_xyz(PointCloud2 & cloud, const RigidTransform & tf)
{
  CloudTransformResult result;

  if (cloud.is_bigendian) {
    result.error = "big-endian point data is not supported";
    return result;
  }

  const auto tx = field_type(cloud, "x");
  const auto ty = field_type(cloud, "y");
  const auto tz = field_type(cloud, "z");
  if (!tx || !ty || !tz) {
    result.error = "cloud is missing x/y/z fields (or a coordinate field has count != 1)";
    return result;
  }
  if (*tx != *ty || *tx != *tz) {
    result.error = "x/y/z fields must share the same datatype";
    return result;
  }
  if (*tx != PointFieldType::kFloat32 && *tx != PointFieldType::kFloat64) {
    result.error = "x/y/z fields must be FLOAT32 or FLOAT64";
    return result;
  }

  XyzLayout layout;
  layout.x_off = *cloud.field_offset("x");
  layout.y_off = *cloud.field_offset("y");
  layout.z_off = *cloud.field_offset("z");
  layout.type = *tx;

  if (cloud.point_step == 0) {
    result.error = "cloud has point_step == 0";
    return result;
  }
  const std::size_t num_points =
    static_cast<std::size_t>(cloud.height) * static_cast<std::size_t>(cloud.width);
  const std::size_t need = num_points * cloud.point_step;
  if (cloud.data.size() < need) {
    result.error = "cloud data is shorter than height * width * point_step";
    return result;
  }

  // An identity transform leaves every coordinate unchanged; skip the sweep so
  // the reference cloud (frame == target) costs nothing.
  if (tf.is_identity()) {
    result.ok = true;
    return result;
  }

  const bool f64 = layout.type == PointFieldType::kFloat64;
  const auto & r = tf.rotation;
  const auto & t = tf.translation;

  for (std::size_t i = 0; i < num_points; ++i) {
    std::byte * base = cloud.data.data() + i * cloud.point_step;
    std::byte * px = base + layout.x_off;
    std::byte * py = base + layout.y_off;
    std::byte * pz = base + layout.z_off;

    const double x = f64 ? load<double>(px) : load<float>(px);
    const double y = f64 ? load<double>(py) : load<float>(py);
    const double z = f64 ? load<double>(pz) : load<float>(pz);

    // Non-finite returns (no-reflection placeholders) carry no position; leave
    // their bytes verbatim so a downstream is_dense == false stays meaningful.
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }

    const double nx = r[0] * x + r[1] * y + r[2] * z + t[0];
    const double ny = r[3] * x + r[4] * y + r[5] * z + t[1];
    const double nz = r[6] * x + r[7] * y + r[8] * z + t[2];

    if (f64) {
      store<double>(px, nx);
      store<double>(py, ny);
      store<double>(pz, nz);
    } else {
      store<float>(px, nx);
      store<float>(py, ny);
      store<float>(pz, nz);
    }
  }

  result.ok = true;
  return result;
}

}  // namespace bagwiz::core::pointcloud
