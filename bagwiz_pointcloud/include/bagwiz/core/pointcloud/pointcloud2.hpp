// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__POINTCLOUD__POINTCLOUD2_HPP_
#define BAGWIZ__CORE__POINTCLOUD__POINTCLOUD2_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace bagwiz::core::pointcloud
{

enum class PointFieldType : std::uint8_t {
  kInt8 = 1,
  kUint8 = 2,
  kInt16 = 3,
  kUint16 = 4,
  kInt32 = 5,
  kUint32 = 6,
  kFloat32 = 7,
  kFloat64 = 8,
};

struct PointField
{
  std::string name;
  std::uint32_t offset = 0;
  PointFieldType datatype = PointFieldType::kFloat32;
  std::uint32_t count = 1;
};

// Size in bytes of one element of a `PointFieldType`.
[[nodiscard]] std::size_t datatype_size(PointFieldType datatype);

// The PointCloud2 metadata that precedes the point payload on the wire: the
// header stamp plus the field layout. Parsing this alone skips the (potentially
// large) point-data copy, so callers that only need timing or field layout can
// avoid materialising the whole cloud.
struct PointCloud2Header
{
  std::int64_t timestamp_ns = 0;
  std::string frame_id;
  std::uint32_t height = 0;
  std::uint32_t width = 0;
  std::vector<PointField> fields;
  bool is_bigendian = false;
  std::uint32_t point_step = 0;
  std::uint32_t row_step = 0;

  std::optional<std::uint32_t> field_offset(const std::string & name) const;
};

struct PointCloud2
{
  std::int64_t timestamp_ns = 0;
  std::string frame_id;
  std::uint32_t height = 0;
  std::uint32_t width = 0;
  std::vector<PointField> fields;
  bool is_bigendian = false;
  std::uint32_t point_step = 0;
  std::uint32_t row_step = 0;
  std::vector<std::byte> data;
  bool is_dense = false;

  std::optional<std::uint32_t> field_offset(const std::string & name) const;
};

struct PointCloud2HeaderResult
{
  std::optional<PointCloud2Header> header;
  std::string error;

  [[nodiscard]] bool ok() const noexcept { return header.has_value() && error.empty(); }
};

struct PointCloud2Result
{
  std::optional<PointCloud2> cloud;
  std::string error;

  [[nodiscard]] bool ok() const noexcept { return cloud.has_value() && error.empty(); }
};

// Parse only the header (stamp + field layout), stopping before the point data.
// Cheap regardless of cloud size: no point bytes are read or copied.
[[nodiscard]] PointCloud2HeaderResult parse_pointcloud2_header(std::span<const std::byte> payload);

[[nodiscard]] PointCloud2Result parse_pointcloud2(std::span<const std::byte> payload);

// Serialize a PointCloud2 back to a plain little-endian CDR-1 payload (the
// inverse of parse_pointcloud2). The result round-trips through
// parse_pointcloud2 field-for-field and byte-for-byte for the point data. The
// output always uses little-endian encapsulation regardless of the host, so it
// is deterministic; `cloud.is_bigendian` is written as the field value but the
// point bytes are emitted verbatim (callers must ensure they are little-endian).
[[nodiscard]] std::vector<std::byte> serialize_pointcloud2(const PointCloud2 & cloud);

}  // namespace bagwiz::core::pointcloud

#endif  // BAGWIZ__CORE__POINTCLOUD__POINTCLOUD2_HPP_
