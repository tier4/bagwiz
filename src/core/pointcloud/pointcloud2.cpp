// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/pointcloud2.hpp"

#include "bagwiz/core/cdr_walker/cdr_reader.hpp"

#include <cstdint>
#include <exception>
#include <span>
#include <string>

namespace bagwiz::core::pointcloud
{

std::optional<std::uint32_t> PointCloud2::field_offset(const std::string & name) const
{
  for (const auto & f : fields) {
    if (f.name == name) {
      return f.offset;
    }
  }
  return std::nullopt;
}

// sensor_msgs/msg/PointCloud2 CDR layout (CDR-1, as written by Fast/Cyclone DDS):
//
//   std_msgs/Header header
//     builtin_interfaces/Time stamp { int32 sec; uint32 nanosec; }
//     string frame_id
//   uint32 height
//   uint32 width
//   PointField[] fields
//     string name
//     uint32 offset
//     uint8  datatype
//     uint32 count
//   bool   is_bigendian
//   uint32 point_step
//   uint32 row_step
//   uint8[] data            // length-prefixed; point_step * height * width bytes
//   bool   is_dense
//
// CdrReader::read_bytes returns a zero-copy view into the CDR payload, which is
// then copied into result.cloud->data so the returned PointCloud2 owns its bytes.
// This avoids materialising a large point cloud element-by-element while still
// giving the caller an independent data vector.
PointCloud2Result parse_pointcloud2(std::span<const std::byte> payload)
{
  PointCloud2Result result;
  try {
    cdr_walker::CdrReader reader(payload);

    const std::int32_t sec = reader.read_i32();
    const std::uint32_t nanosec = reader.read_u32();
    result.cloud.emplace();
    result.cloud->timestamp_ns = static_cast<std::int64_t>(sec) * 1'000'000'000LL + nanosec;
    result.cloud->frame_id = reader.read_string();
    result.cloud->height = reader.read_u32();
    result.cloud->width = reader.read_u32();

    const std::uint32_t field_count = reader.read_sequence_length();
    result.cloud->fields.resize(field_count);
    for (std::uint32_t i = 0; i < field_count; ++i) {
      auto & f = result.cloud->fields[i];
      f.name = reader.read_string();
      f.offset = reader.read_u32();
      f.datatype = static_cast<PointFieldType>(reader.read_u8());
      f.count = reader.read_u32();
    }

    result.cloud->is_bigendian = reader.read_bool();
    result.cloud->point_step = reader.read_u32();
    result.cloud->row_step = reader.read_u32();

    const std::uint32_t data_len = reader.read_sequence_length();
    const auto data_span = reader.read_bytes(data_len);
    result.cloud->data.assign(data_span.begin(), data_span.end());

    result.cloud->is_dense = reader.read_bool();
  } catch (const std::exception & e) {
    result.cloud.reset();
    result.error = std::string("failed to parse sensor_msgs/msg/PointCloud2 payload: ") + e.what();
  }
  return result;
}

}  // namespace bagwiz::core::pointcloud
