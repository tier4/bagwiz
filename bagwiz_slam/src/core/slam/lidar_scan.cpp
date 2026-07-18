// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/lidar_scan.hpp"

#include "bagwiz/core/pointcloud/point_time.hpp"
#include "bagwiz/core/pointcloud/pointcloud2.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

namespace bagwiz::core::slam
{
namespace
{
using core::pointcloud::PointCloud2;
using core::pointcloud::PointField;
using core::pointcloud::PointFieldType;

const PointField * find_field(const PointCloud2 & cloud, const std::string & name)
{
  for (const auto & f : cloud.fields) {
    if (f.name == name) {
      return &f;
    }
  }
  return nullptr;
}

// Copy `sizeof(T)` bytes out of the (possibly unaligned) cloud buffer. memcpy
// keeps this free of alignment / strict-aliasing UB.
template <typename T>
T load(const std::vector<std::byte> & data, std::size_t byte_offset)
{
  T value{};
  std::memcpy(&value, data.data() + byte_offset, sizeof(T));
  return value;
}

// Read one field of `point_idx` as a double, widening from its wire type.
double read_as_double(
  const PointCloud2 & cloud, std::uint32_t point_idx, std::uint32_t offset, PointFieldType dt)
{
  const std::size_t base = static_cast<std::size_t>(point_idx) * cloud.point_step + offset;
  switch (dt) {
    case PointFieldType::kInt8:
      return static_cast<double>(load<std::int8_t>(cloud.data, base));
    case PointFieldType::kUint8:
      return static_cast<double>(load<std::uint8_t>(cloud.data, base));
    case PointFieldType::kInt16:
      return static_cast<double>(load<std::int16_t>(cloud.data, base));
    case PointFieldType::kUint16:
      return static_cast<double>(load<std::uint16_t>(cloud.data, base));
    case PointFieldType::kInt32:
      return static_cast<double>(load<std::int32_t>(cloud.data, base));
    case PointFieldType::kUint32:
      return static_cast<double>(load<std::uint32_t>(cloud.data, base));
    case PointFieldType::kFloat32:
      return static_cast<double>(load<float>(cloud.data, base));
    case PointFieldType::kFloat64:
      return load<double>(cloud.data, base);
  }
  return 0.0;
}

bool is_float(PointFieldType dt) noexcept
{
  return dt == PointFieldType::kFloat32 || dt == PointFieldType::kFloat64;
}

// True when a field's `offset + datatype_size` stays inside point_step.
bool field_fits(const PointCloud2 & cloud, const PointField & field) noexcept
{
  return static_cast<std::size_t>(field.offset) + datatype_size(field.datatype) <= cloud.point_step;
}

}  // namespace

LidarScanResult to_lidar_scan(const PointCloud2 & cloud, const std::string & intensity_field)
{
  LidarScanResult result;

  if (cloud.is_bigendian) {
    result.error = "big-endian PointCloud2 point data is not supported";
    return result;
  }

  const PointField * fx = find_field(cloud, "x");
  const PointField * fy = find_field(cloud, "y");
  const PointField * fz = find_field(cloud, "z");
  if (fx == nullptr || fy == nullptr || fz == nullptr) {
    result.error = "PointCloud2 is missing one of the x/y/z fields";
    return result;
  }
  if (!is_float(fx->datatype) || !is_float(fy->datatype) || !is_float(fz->datatype)) {
    result.error = "PointCloud2 x/y/z fields must be FLOAT32 or FLOAT64";
    return result;
  }

  const PointField * fi = find_field(cloud, intensity_field);
  const auto ft = core::pointcloud::find_point_time_field(cloud);
  const bool use_time = ft.has_value();

  if (cloud.point_step == 0) {
    result.error = "PointCloud2 point_step is zero";
    return result;
  }
  for (const PointField * f : {fx, fy, fz, fi}) {
    if (f != nullptr && !field_fits(cloud, *f)) {
      result.error = "a PointCloud2 field extends past point_step";
      return result;
    }
  }
  if (
    use_time &&
    static_cast<std::size_t>(ft->offset) + datatype_size(ft->datatype) > cloud.point_step) {
    result.error = "a PointCloud2 field extends past point_step";
    return result;
  }

  const std::size_t num_points = static_cast<std::size_t>(cloud.width) * cloud.height;
  if (cloud.data.size() < num_points * cloud.point_step) {
    result.error = "PointCloud2 data buffer is smaller than width*height*point_step";
    return result;
  }

  LidarScan scan;
  scan.stamp_ns = cloud.timestamp_ns;
  scan.frame_id = cloud.frame_id;
  scan.points.reserve(num_points);
  if (fi != nullptr) {
    scan.intensities.reserve(num_points);
  }
  if (use_time) {
    scan.times.reserve(num_points);
    scan.has_per_point_time = true;
  }

  for (std::uint32_t i = 0; i < num_points; ++i) {
    scan.points.push_back(
      {read_as_double(cloud, i, fx->offset, fx->datatype),
       read_as_double(cloud, i, fy->offset, fy->datatype),
       read_as_double(cloud, i, fz->offset, fz->datatype)});
    if (fi != nullptr) {
      scan.intensities.push_back(read_as_double(cloud, i, fi->offset, fi->datatype));
    }
    if (use_time) {
      const std::size_t base = static_cast<std::size_t>(i) * cloud.point_step + ft->offset;
      scan.times.push_back(core::pointcloud::point_time_seconds(cloud.data.data() + base, *ft));
    }
  }

  result.scan = std::move(scan);
  return result;
}

}  // namespace bagwiz::core::slam
