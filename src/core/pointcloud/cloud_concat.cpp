// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/cloud_concat.hpp"

#include "bagwiz/core/pointcloud/point_time.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace bagwiz::core::pointcloud
{

namespace
{

// Identical field layout: same fields (name/offset/datatype/count, order) and
// the same point_step. frame_id / width / height / stamp are allowed to differ.
bool same_layout(const PointCloud2 & a, const PointCloud2 & b)
{
  if (a.point_step != b.point_step || a.fields.size() != b.fields.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.fields.size(); ++i) {
    const auto & fa = a.fields[i];
    const auto & fb = b.fields[i];
    if (
      fa.name != fb.name || fa.offset != fb.offset || fa.datatype != fb.datatype ||
      fa.count != fb.count) {
      return false;
    }
  }
  return true;
}

std::size_t point_count(const PointCloud2 & c)
{
  return static_cast<std::size_t>(c.height) * static_cast<std::size_t>(c.width);
}

// Decide whether a per-point time field is absolute (encodes epoch time) rather
// than relative to the message header.stamp, using the first finite sample
// across the inputs. Absolute values sit next to the header stamp (~1.7e9 s);
// relative ones sit near 0. The gap is enormous, so "closer to the header than
// to zero" cleanly separates them.
bool detect_absolute_time(std::span<const ConcatInput> inputs, const PointTimeField & tf)
{
  for (const auto & in : inputs) {
    const PointCloud2 & c = *in.cloud;
    const std::size_t n = point_count(c);
    const double header_sec = static_cast<double>(in.header_stamp_ns) * 1e-9;
    for (std::size_t i = 0; i < n; ++i) {
      const std::byte * ptr = c.data.data() + i * c.point_step + tf.offset;
      const double t = point_time_seconds(ptr, tf);
      if (std::isfinite(t)) {
        return std::abs(t - header_sec) < std::abs(t);
      }
    }
  }
  // No finite sample anywhere: nothing to base a decision on. Treat as absolute
  // (verbatim copy) — re-basing an unknown convention would risk corruption.
  return true;
}

// Re-base a header-relative FLOAT32/FLOAT64 per-point time block in place by
// `delta_ns` so the point's absolute time is preserved under the new output
// header. (UINT32 time is converted to FLOAT32 in concat_clouds, not here.)
bool rebase_time(
  std::vector<std::byte> & block, std::size_t num_points, std::uint32_t point_step,
  const PointTimeField & tf, std::int64_t delta_ns, std::string & error)
{
  const double delta_sec = static_cast<double>(delta_ns) * 1e-9;
  for (std::size_t i = 0; i < num_points; ++i) {
    std::byte * ptr = block.data() + i * point_step + tf.offset;
    switch (tf.datatype) {
      case PointFieldType::kFloat32: {
        float v = 0.0F;
        std::memcpy(&v, ptr, sizeof(v));
        if (std::isfinite(v)) {
          v = static_cast<float>(static_cast<double>(v) + delta_sec);
          std::memcpy(ptr, &v, sizeof(v));
        }
        break;
      }
      case PointFieldType::kFloat64: {
        double v = 0.0;
        std::memcpy(&v, ptr, sizeof(v));
        if (std::isfinite(v)) {
          v += delta_sec;
          std::memcpy(ptr, &v, sizeof(v));
        }
        break;
      }
      default:
        // UINT32 time is converted to FLOAT32 in concat_clouds before this runs,
        // so re-basing in place only ever handles the float encodings here.
        error = "unsupported per-point time datatype for re-basing (need FLOAT32/FLOAT64)";
        return false;
    }
  }
  return true;
}

}  // namespace

ConcatResult concat_clouds(
  std::span<const ConcatInput> inputs, std::int64_t out_stamp_ns, std::string out_frame)
{
  ConcatResult result;

  if (inputs.empty()) {
    result.error = "no input clouds to concatenate";
    return result;
  }
  for (const auto & in : inputs) {
    if (in.cloud == nullptr) {
      result.error = "null input cloud";
      return result;
    }
    if (in.cloud->is_bigendian) {
      result.error = "big-endian point data is not supported";
      return result;
    }
  }

  const PointCloud2 & first = *inputs[0].cloud;
  if (first.point_step == 0) {
    result.error = "cloud has point_step == 0";
    return result;
  }
  for (std::size_t k = 1; k < inputs.size(); ++k) {
    if (!same_layout(first, *inputs[k].cloud)) {
      result.error =
        "input clouds have mismatched PointField layouts; concat requires identical fields and "
        "point_step across all --input-topics";
      return result;
    }
  }

  const auto time_field = find_point_time_field(first);
  // A UINT32 header-relative time (unsigned ns) cannot hold the negative value
  // re-basing produces when an input is earlier than out_stamp, so its output
  // field is emitted as FLOAT32 seconds (same 4-byte width; the value may be
  // negative). FLOAT32/FLOAT64 times are already signed and are re-based in place.
  // UINT32 is always relative (it cannot encode an epoch-scale absolute stamp), so
  // it needs no absolute-vs-relative test.
  const bool time_u32_to_f32 =
    time_field.has_value() && time_field->datatype == PointFieldType::kUint32;

  // Validate before any byte-level access (detect_absolute_time below and the
  // per-point loops): the time field must fit inside a point, and every input's
  // data buffer must hold height*width*point_step bytes. parse_pointcloud2 does
  // not validate field offsets against point_step.
  if (
    time_field.has_value() &&
    static_cast<std::size_t>(time_field->offset) + datatype_size(time_field->datatype) >
      first.point_step) {
    result.error = "per-point time field extends past point_step";
    return result;
  }
  for (const auto & in : inputs) {
    const PointCloud2 & c = *in.cloud;
    if (c.data.size() < point_count(c) * c.point_step) {
      result.error = "input cloud data is shorter than height * width * point_step";
      return result;
    }
  }

  const bool has_relative_time =
    time_field.has_value() && !time_u32_to_f32 && !detect_absolute_time(inputs, *time_field);

  PointCloud2 out;
  out.timestamp_ns = out_stamp_ns;
  out.frame_id = std::move(out_frame);
  out.height = 1;
  out.is_bigendian = false;
  out.fields = first.fields;
  out.point_step = first.point_step;
  if (time_u32_to_f32) {
    for (auto & f : out.fields) {
      if (f.offset == time_field->offset) {
        f.datatype = PointFieldType::kFloat32;
      }
    }
  }

  std::size_t total_points = 0;
  bool is_dense = true;
  for (const auto & in : inputs) {
    const PointCloud2 & c = *in.cloud;
    const std::size_t n = point_count(c);
    const std::size_t need = n * c.point_step;

    // Copy only the meaningful point bytes (n * point_step); an organized cloud
    // is flattened, and any trailing slack in c.data is dropped.
    std::vector<std::byte> block(
      c.data.begin(), c.data.begin() + static_cast<std::ptrdiff_t>(need));

    if (time_u32_to_f32) {
      // Rewrite each UINT32-ns time slot in place as FLOAT32 seconds re-based to
      // out_stamp: t' = t_ns/1e9 + (header_stamp_k - out_stamp)/1e9, so out_stamp +
      // t' == header_stamp_k + t_ns (t' may be negative for an earlier input).
      const double delta_sec = static_cast<double>(in.header_stamp_ns - out_stamp_ns) * 1e-9;
      for (std::size_t i = 0; i < n; ++i) {
        std::byte * ptr = block.data() + i * c.point_step + time_field->offset;
        std::uint32_t t_ns = 0;
        std::memcpy(&t_ns, ptr, sizeof(t_ns));
        const float t_sec = static_cast<float>(static_cast<double>(t_ns) * 1e-9 + delta_sec);
        std::memcpy(ptr, &t_sec, sizeof(t_sec));
      }
    } else if (has_relative_time) {
      const std::int64_t delta_ns = in.header_stamp_ns - out_stamp_ns;
      if (delta_ns != 0) {
        if (!rebase_time(block, n, out.point_step, *time_field, delta_ns, result.error)) {
          return result;
        }
      }
    }

    out.data.insert(out.data.end(), block.begin(), block.end());
    total_points += n;
    is_dense = is_dense && c.is_dense;
  }

  out.width = static_cast<std::uint32_t>(total_points);
  out.row_step = out.point_step * out.width;
  out.is_dense = is_dense;

  result.cloud = std::move(out);
  return result;
}

}  // namespace bagwiz::core::pointcloud
