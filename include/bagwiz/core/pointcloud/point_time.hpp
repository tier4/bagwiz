// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__POINTCLOUD__POINT_TIME_HPP_
#define BAGWIZ__CORE__POINTCLOUD__POINT_TIME_HPP_

#include "bagwiz/core/pointcloud/pointcloud2.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>

namespace bagwiz::core::pointcloud
{

// A recognised per-point time field: its byte offset within a point and its
// datatype (one of UINT32 / FLOAT32 / FLOAT64).
struct PointTimeField
{
  std::uint32_t offset = 0;
  PointFieldType datatype = PointFieldType::kFloat32;
};

// The per-point time field bagwiz recognises, or nullopt. Shared by `map slam`
// (LiDAR scan extraction) and `pcd concat` so both agree on what counts as a
// per-point time. Following glim's convention, a field qualifies when:
//   - its name is one of "t", "time", "time_stamp", "timestamp" (tried in this
//     precedence order — the first qualifying field wins),
//   - its count is 1, and
//   - its datatype is UINT32 (nanoseconds) or FLOAT32 / FLOAT64 (seconds).
// A field that fails any check is ignored (the cloud is treated as having no
// per-point time); the search falls through to the next name.
[[nodiscard]] std::optional<PointTimeField> find_point_time_field(const PointCloud2 & cloud);

// The per-point time at `field_bytes` (a pointer to the field's first byte for
// one point) as seconds: a UINT32 value is divided by 1e9, a FLOAT32 / FLOAT64
// value is taken as-is. Whether those seconds are relative (to header.stamp) or
// absolute is not decided here. The caller must ensure the field's bytes are in
// bounds. A non-finite FLOAT value is returned as-is (callers that sample the
// field can test std::isfinite).
[[nodiscard]] double point_time_seconds(
  const std::byte * field_bytes, const PointTimeField & field);

}  // namespace bagwiz::core::pointcloud

#endif  // BAGWIZ__CORE__POINTCLOUD__POINT_TIME_HPP_
