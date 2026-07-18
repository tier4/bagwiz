// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__POINTCLOUD__CLOUD_CONCAT_HPP_
#define BAGWIZ__CORE__POINTCLOUD__CLOUD_CONCAT_HPP_

#include "bagwiz/core/pointcloud/pointcloud2.hpp"

#include <cstdint>
#include <optional>
#include <span>
#include <string>

namespace bagwiz::core::pointcloud
{

// One already-frame-transformed input cloud for concatenation, paired with its
// ORIGINAL header.stamp. The stamp is used ONLY to preserve per-point absolute
// time when the time field is header-relative (see concat_clouds); it never
// changes the output header.stamp. The `--stamp-offset` matching aid must NOT
// be folded into this value — pass the real message stamp.
struct ConcatInput
{
  const PointCloud2 * cloud = nullptr;  // transformed to the target frame
  std::int64_t header_stamp_ns = 0;     // the source message's real header.stamp
};

struct ConcatResult
{
  std::optional<PointCloud2> cloud;
  std::string error;

  [[nodiscard]] bool ok() const noexcept { return cloud.has_value() && error.empty(); }
};

// Concatenate `inputs` (order preserved) into ONE unorganized PointCloud2 with
// header stamp `out_stamp_ns` and frame `out_frame`.
//
// Requires a non-empty `inputs` whose clouds share an identical field layout
// (fields name/offset/datatype/count, point_step) and are all little-endian;
// otherwise an error is returned. The result:
//   - height = 1, width = Σ (height_k * width_k), row_step = point_step * width
//   - data   = each input's point bytes appended in order
//   - is_dense = logical AND of the inputs
//   - other fields (intensity/ring/…) copied verbatim
//
// Per-point time preservation: if the layout has a per-point time field
// (name t / time / time_stamp / timestamp), each point's ORIGINAL ABSOLUTE
// acquisition time is preserved. An absolute-encoded field is copied verbatim; a
// header-relative FLOAT field is re-based in place by (header_stamp_ns[k] -
// out_stamp_ns); a header-relative UINT32 field is emitted as FLOAT32 seconds
// (same 4-byte width, point_step unchanged) and re-based, so the value stays
// representable even when negative (an input earlier than out_stamp). In every
// case out_stamp + t' == header_stamp_k + t.
ConcatResult concat_clouds(
  std::span<const ConcatInput> inputs, std::int64_t out_stamp_ns, std::string out_frame);

}  // namespace bagwiz::core::pointcloud

#endif  // BAGWIZ__CORE__POINTCLOUD__CLOUD_CONCAT_HPP_
