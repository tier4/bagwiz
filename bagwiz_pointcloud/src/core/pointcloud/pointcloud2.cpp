// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/pointcloud2.hpp"

#include "bagwiz/core/cdr_walker/cdr_reader.hpp"
#include "bagwiz/core/cdr_walker/cdr_writer.hpp"

#include <cstddef>
#include <cstdint>
#include <exception>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace bagwiz::core::pointcloud
{

namespace
{

std::optional<std::uint32_t> find_field_offset(
  const std::vector<PointField> & fields, const std::string & name)
{
  for (const auto & f : fields) {
    if (f.name == name) {
      return f.offset;
    }
  }
  return std::nullopt;
}

// Read the header (stamp .. row_step) from `reader`, leaving it positioned at
// the point-data sequence-length prefix. No point bytes are read.
PointCloud2Header read_header(cdr_walker::CdrReader & reader)
{
  PointCloud2Header header;

  const std::int32_t sec = reader.read_i32();
  const std::uint32_t nanosec = reader.read_u32();
  header.timestamp_ns = static_cast<std::int64_t>(sec) * 1'000'000'000LL + nanosec;
  header.frame_id = reader.read_string();
  header.height = reader.read_u32();
  header.width = reader.read_u32();

  const std::uint32_t field_count = reader.read_sequence_length();
  header.fields.resize(field_count);
  for (std::uint32_t i = 0; i < field_count; ++i) {
    auto & f = header.fields[i];
    f.name = reader.read_string();
    f.offset = reader.read_u32();
    f.datatype = static_cast<PointFieldType>(reader.read_u8());
    f.count = reader.read_u32();
  }

  header.is_bigendian = reader.read_bool();
  header.point_step = reader.read_u32();
  header.row_step = reader.read_u32();
  return header;
}

}  // namespace

std::optional<std::uint32_t> PointCloud2Header::field_offset(const std::string & name) const
{
  return find_field_offset(fields, name);
}

std::optional<std::uint32_t> PointCloud2::field_offset(const std::string & name) const
{
  return find_field_offset(fields, name);
}

std::size_t datatype_size(PointFieldType datatype)
{
  switch (datatype) {
    case PointFieldType::kInt8:
    case PointFieldType::kUint8:
      return 1;
    case PointFieldType::kInt16:
    case PointFieldType::kUint16:
      return 2;
    case PointFieldType::kInt32:
    case PointFieldType::kUint32:
    case PointFieldType::kFloat32:
      return 4;
    case PointFieldType::kFloat64:
      return 8;
  }
  return 0;
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
// read_header() decodes everything up to (but not including) the point data, so
// this parses only the metadata and never touches the point bytes.
PointCloud2HeaderResult parse_pointcloud2_header(std::span<const std::byte> payload)
{
  PointCloud2HeaderResult result;
  try {
    cdr_walker::CdrReader reader(payload);
    result.header = read_header(reader);
  } catch (const std::exception & e) {
    result.header.reset();
    result.error = std::string("failed to parse sensor_msgs/msg/PointCloud2 payload: ") + e.what();
  }
  return result;
}

// Parse the full message. The header is decoded by read_header(); the point data
// that follows is a zero-copy view from CdrReader::read_bytes(), copied into
// result.cloud->data so the returned PointCloud2 owns its bytes.
PointCloud2Result parse_pointcloud2(std::span<const std::byte> payload)
{
  PointCloud2Result result;
  try {
    cdr_walker::CdrReader reader(payload);

    PointCloud2Header header = read_header(reader);
    result.cloud.emplace();
    auto & cloud = *result.cloud;
    cloud.timestamp_ns = header.timestamp_ns;
    cloud.frame_id = std::move(header.frame_id);
    cloud.height = header.height;
    cloud.width = header.width;
    cloud.fields = std::move(header.fields);
    cloud.is_bigendian = header.is_bigendian;
    cloud.point_step = header.point_step;
    cloud.row_step = header.row_step;

    const std::uint32_t data_len = reader.read_sequence_length();
    const auto data_span = reader.read_bytes(data_len);
    cloud.data.assign(data_span.begin(), data_span.end());

    cloud.is_dense = reader.read_bool();
  } catch (const std::exception & e) {
    result.cloud.reset();
    result.error = std::string("failed to parse sensor_msgs/msg/PointCloud2 payload: ") + e.what();
  }
  return result;
}

// Inverse of parse_pointcloud2: emit the CDR-1 layout documented above via
// CdrWriter (which mirrors CdrReader's alignment), so parse(serialize(c)) == c.
std::vector<std::byte> serialize_pointcloud2(const PointCloud2 & cloud)
{
  cdr_walker::CdrWriter writer;

  // builtin_interfaces/Time: int32 sec + uint32 nanosec, recomposed by the
  // reader as sec * 1e9 + nanosec. ROS stamps are non-negative.
  const std::int64_t ts = cloud.timestamp_ns;
  writer.write_i32(static_cast<std::int32_t>(ts / 1'000'000'000LL));
  writer.write_u32(static_cast<std::uint32_t>(ts % 1'000'000'000LL));
  writer.write_string(cloud.frame_id);
  writer.write_u32(cloud.height);
  writer.write_u32(cloud.width);

  writer.write_sequence_length(static_cast<std::uint32_t>(cloud.fields.size()));
  for (const auto & f : cloud.fields) {
    writer.write_string(f.name);
    writer.write_u32(f.offset);
    writer.write_u8(static_cast<std::uint8_t>(f.datatype));
    writer.write_u32(f.count);
  }

  writer.write_bool(cloud.is_bigendian);
  writer.write_u32(cloud.point_step);
  writer.write_u32(cloud.row_step);

  writer.write_sequence_length(static_cast<std::uint32_t>(cloud.data.size()));
  writer.write_bytes(cloud.data);

  writer.write_bool(cloud.is_dense);
  return writer.take();
}

}  // namespace bagwiz::core::pointcloud
