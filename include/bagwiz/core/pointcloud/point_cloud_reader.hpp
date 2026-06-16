// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__POINTCLOUD__POINT_CLOUD_READER_HPP_
#define BAGWIZ__CORE__POINTCLOUD__POINT_CLOUD_READER_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>

namespace bagwiz::core::pointcloud
{

// A view into a decoded sensor_msgs/msg/PointCloud2 payload. The `data` span
// references the original payload buffer and is not owned by this struct.
struct PointCloudView {
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::string frame_id;
  std::uint32_t point_step = 0;
  std::uint32_t row_step = 0;
  bool is_dense = false;
  bool is_bigendian = false;

  std::optional<std::size_t> x_offset;
  std::optional<std::size_t> y_offset;
  std::optional<std::size_t> z_offset;
  std::optional<std::size_t> intensity_offset;
  std::uint8_t intensity_datatype = 0;

  std::span<const std::byte> data;
};

// Outcome of extract_point_cloud(). On success `ok` is true and `view` holds
// the decoded metadata and a non-owning view of the point data; on failure
// `ok` is false and `error` carries a human-readable reason. Never throws.
struct PointCloudResult {
  bool ok = false;
  std::string error;
  PointCloudView view;
};

// Parse a CDR-serialized sensor_msgs/msg/PointCloud2 payload and extract the
// x/y/z field offsets and an optional intensity field. Requires x/y/z to be
// present and of type FLOAT32 (datatype 7).
[[nodiscard]] PointCloudResult extract_point_cloud(std::span<const std::byte> payload);

// Read a FLOAT32 field from `view.data` for the given point index.
[[nodiscard]] float read_float_field(
  const PointCloudView & view, std::size_t point_index, std::size_t offset);

// Read the intensity field for the given point index and normalize integer
// types (UINT8 / UINT16 in the task's datatype convention) to [0, 1].
[[nodiscard]] std::optional<float> read_intensity(
  const PointCloudView & view, std::size_t point_index);

// Return true if x/y/z for the given point index are all finite floats.
[[nodiscard]] bool is_valid_point(const PointCloudView & view, std::size_t point_index);

}  // namespace bagwiz::core::pointcloud

#endif  // BAGWIZ__CORE__POINTCLOUD__POINT_CLOUD_READER_HPP_
