// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/point_time.hpp"

#include <array>
#include <cstring>

namespace bagwiz::core::pointcloud
{

namespace
{

// Per-point time field names in glim's precedence order.
constexpr std::array<const char *, 4> kTimeFieldNames{"t", "time", "time_stamp", "timestamp"};

bool is_supported(PointFieldType datatype)
{
  return datatype == PointFieldType::kUint32 || datatype == PointFieldType::kFloat32 ||
         datatype == PointFieldType::kFloat64;
}

}  // namespace

std::optional<PointTimeField> find_point_time_field(const PointCloud2 & cloud)
{
  for (const auto * const name : kTimeFieldNames) {
    for (const auto & f : cloud.fields) {
      if (f.name == name && f.count == 1 && is_supported(f.datatype)) {
        return PointTimeField{f.offset, f.datatype};
      }
    }
  }
  return std::nullopt;
}

double point_time_seconds(const std::byte * field_bytes, const PointTimeField & field)
{
  switch (field.datatype) {
    case PointFieldType::kUint32: {
      std::uint32_t v = 0;
      std::memcpy(&v, field_bytes, sizeof(v));
      return static_cast<double>(v) / 1e9;  // nanoseconds -> seconds
    }
    case PointFieldType::kFloat32: {
      float v = 0.0F;
      std::memcpy(&v, field_bytes, sizeof(v));
      return static_cast<double>(v);
    }
    case PointFieldType::kFloat64: {
      double v = 0.0;
      std::memcpy(&v, field_bytes, sizeof(v));
      return v;
    }
    default:
      return 0.0;  // unreachable: find_point_time_field only returns supported datatypes
  }
}

}  // namespace bagwiz::core::pointcloud
