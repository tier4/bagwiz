// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/fetcher.hpp"

#include "bagwiz/core/pointcloud/pointcloud2.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace bagwiz::core::pointcloud
{

namespace
{

const PointField * find_point_field(const PointCloud2 & cloud, const std::string & name)
{
  for (const auto & f : cloud.fields) {
    if (f.name == name) {
      return &f;
    }
  }
  return nullptr;
}

float read_point_field(
  const PointCloud2 & cloud, std::uint32_t point_idx, std::uint32_t offset, PointFieldType type)
{
  const std::byte * base = cloud.data.data() + point_idx * cloud.point_step + offset;
  switch (type) {
    case PointFieldType::kFloat32:
      return *reinterpret_cast<const float *>(base);
    case PointFieldType::kFloat64:
      return static_cast<float>(*reinterpret_cast<const double *>(base));
    case PointFieldType::kInt8:
      return static_cast<float>(*reinterpret_cast<const std::int8_t *>(base));
    case PointFieldType::kUint8:
      return static_cast<float>(*reinterpret_cast<const std::uint8_t *>(base));
    case PointFieldType::kInt16:
      return static_cast<float>(*reinterpret_cast<const std::int16_t *>(base));
    case PointFieldType::kUint16:
      return static_cast<float>(*reinterpret_cast<const std::uint16_t *>(base));
    case PointFieldType::kInt32:
      return static_cast<float>(*reinterpret_cast<const std::int32_t *>(base));
    case PointFieldType::kUint32:
      return static_cast<float>(*reinterpret_cast<const std::uint32_t *>(base));
  }
  return 0.0f;
}

float compute_property_value(
  const PointCloud2 & cloud, std::uint32_t point_idx, const PointField * field_x,
  const PointField * field_y, const PointField * field_z, const PointField * field_intensity,
  std::uint32_t off_x, std::uint32_t off_y, std::uint32_t off_z,
  std::optional<std::uint32_t> off_intensity, PointCloudProperty property)
{
  const float px = read_point_field(cloud, point_idx, off_x, field_x->datatype);
  const float py = read_point_field(cloud, point_idx, off_y, field_y->datatype);
  const float pz = read_point_field(cloud, point_idx, off_z, field_z->datatype);

  switch (property) {
    case PointCloudProperty::kX:
      return px;
    case PointCloudProperty::kY:
      return py;
    case PointCloudProperty::kZ:
      return pz;
    case PointCloudProperty::kDistance:
      return std::sqrt(px * px + py * py + pz * pz);
    case PointCloudProperty::kIntensity:
      return read_point_field(cloud, point_idx, *off_intensity, field_intensity->datatype);
  }
  return 0.0f;
}

}  // namespace

std::optional<PointCloudIndex> build_point_cloud_index(
  const std::filesystem::path & input, const std::string & topic, PointCloudProperty property,
  const std::optional<double> & manual_min, const std::optional<double> & manual_max,
  std::string & error)
{
  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(input);
  } catch (const std::exception & e) {
    error = "failed to open '" + input.string() + "': " + e.what();
    return std::nullopt;
  }

  io::ReadFilter filter;
  filter.topics.push_back(topic);
  reader->set_filter(filter);

  const bool need_auto_min = !manual_min.has_value();
  const bool need_auto_max = !manual_max.has_value();
  const bool need_value_scan = need_auto_min || need_auto_max;

  PointCloudIndex result;
  double running_min = std::numeric_limits<double>::infinity();
  double running_max = -std::numeric_limits<double>::infinity();

  const bool need_intensity = (property == PointCloudProperty::kIntensity);

  io::RawMessage raw;
  try {
    while (reader->next(raw)) {
      result.entries.push_back({raw.timestamp_ns});

      // Parse at least the first message to learn field layout, and scan values
      // when an auto range is required.
      if (!need_value_scan && result.has_intensity) {
        continue;
      }

      const auto parsed = parse_pointcloud2(raw.payload);
      if (!parsed.ok()) {
        error = parsed.error;
        return std::nullopt;
      }
      const auto & cloud = *parsed.cloud;

      if (!result.has_intensity) {
        result.has_intensity = cloud.field_offset("intensity").has_value();
      }

      if (!need_value_scan) {
        continue;
      }

      const auto off_x = cloud.field_offset("x");
      const auto off_y = cloud.field_offset("y");
      const auto off_z = cloud.field_offset("z");
      if (!off_x || !off_y || !off_z) {
        error = "point cloud is missing required x/y/z fields";
        return std::nullopt;
      }
      const auto * field_x = find_point_field(cloud, "x");
      const auto * field_y = find_point_field(cloud, "y");
      const auto * field_z = find_point_field(cloud, "z");

      std::optional<std::uint32_t> off_intensity;
      const PointField * field_intensity = nullptr;
      if (need_intensity) {
        off_intensity = cloud.field_offset("intensity");
        if (!off_intensity) {
          error = "point cloud has no intensity field";
          return std::nullopt;
        }
        field_intensity = find_point_field(cloud, "intensity");
      }

      const std::uint32_t n = cloud.height * cloud.width;
      for (std::uint32_t i = 0; i < n; ++i) {
        const float value = compute_property_value(
          cloud, i, field_x, field_y, field_z, field_intensity, *off_x, *off_y, *off_z,
          off_intensity, property);
        running_min = std::min(running_min, static_cast<double>(value));
        running_max = std::max(running_max, static_cast<double>(value));
      }
    }
  } catch (const std::exception & e) {
    error = "error reading point-cloud topic '" + topic + "': " + e.what();
    return std::nullopt;
  }

  if (result.entries.empty()) {
    error = "point-cloud topic '" + topic + "' has no messages";
    return std::nullopt;
  }

  result.property_min = need_auto_min ? running_min : *manual_min;
  result.property_max = need_auto_max ? running_max : *manual_max;
  return result;
}

PropertyRanges::PropertyRanges()
{
  mins.fill(std::numeric_limits<double>::infinity());
  maxs.fill(-std::numeric_limits<double>::infinity());
}

void PropertyRanges::merge(const PropertyRanges & other)
{
  for (std::size_t i = 0; i < kCount; ++i) {
    mins[i] = std::min(mins[i], other.mins[i]);
    maxs[i] = std::max(maxs[i], other.maxs[i]);
  }
  has_intensity = has_intensity || other.has_intensity;
}

std::pair<double, double> PropertyRanges::resolve(PointCloudProperty property) const
{
  const auto i = static_cast<std::size_t>(property);
  // A property that was never observed still holds the +inf/-inf sentinels
  // (min > max); substitute a neutral range so the colour map stays sane.
  if (mins[i] <= maxs[i]) {
    return {mins[i], maxs[i]};
  }
  return {0.0, 1.0};
}

bool accumulate_property_ranges(
  const PointCloud2 & cloud, PropertyRanges & running, std::string & error)
{
  // PropertyRanges is indexed by the raw enum value, so the enum must stay
  // contiguous and exactly cover every slot.
  static_assert(
    static_cast<std::size_t>(PointCloudProperty::kIntensity) + 1 == PropertyRanges::kCount,
    "PropertyRanges::kCount must match the PointCloudProperty enumeration");

  const auto off_x = cloud.field_offset("x");
  const auto off_y = cloud.field_offset("y");
  const auto off_z = cloud.field_offset("z");
  if (!off_x || !off_y || !off_z) {
    error = "point cloud is missing required x/y/z fields";
    return false;
  }
  const auto * field_x = find_point_field(cloud, "x");
  const auto * field_y = find_point_field(cloud, "y");
  const auto * field_z = find_point_field(cloud, "z");

  const auto off_intensity = cloud.field_offset("intensity");
  const PointField * field_intensity =
    off_intensity.has_value() ? find_point_field(cloud, "intensity") : nullptr;
  if (off_intensity.has_value()) {
    running.has_intensity = true;
  }

  constexpr auto x_index = static_cast<std::size_t>(PointCloudProperty::kX);
  constexpr auto y_index = static_cast<std::size_t>(PointCloudProperty::kY);
  constexpr auto z_index = static_cast<std::size_t>(PointCloudProperty::kZ);
  constexpr auto distance_index = static_cast<std::size_t>(PointCloudProperty::kDistance);
  constexpr auto intensity_index = static_cast<std::size_t>(PointCloudProperty::kIntensity);

  auto fold = [](double & lo, double & hi, double value) {
    lo = std::min(lo, value);
    hi = std::max(hi, value);
  };

  const std::uint32_t n = cloud.height * cloud.width;
  for (std::uint32_t i = 0; i < n; ++i) {
    const float px = read_point_field(cloud, i, *off_x, field_x->datatype);
    const float py = read_point_field(cloud, i, *off_y, field_y->datatype);
    const float pz = read_point_field(cloud, i, *off_z, field_z->datatype);
    fold(running.mins[x_index], running.maxs[x_index], static_cast<double>(px));
    fold(running.mins[y_index], running.maxs[y_index], static_cast<double>(py));
    fold(running.mins[z_index], running.maxs[z_index], static_cast<double>(pz));
    const float dist = std::sqrt(px * px + py * py + pz * pz);
    fold(running.mins[distance_index], running.maxs[distance_index], static_cast<double>(dist));
    if (field_intensity != nullptr) {
      const float pi = read_point_field(cloud, i, *off_intensity, field_intensity->datatype);
      fold(running.mins[intensity_index], running.maxs[intensity_index], static_cast<double>(pi));
    }
  }
  return true;
}

std::optional<PointCloudScan> scan_point_cloud(
  const std::filesystem::path & input, const std::string & topic, std::string & error)
{
  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(input);
  } catch (const std::exception & e) {
    error = "failed to open '" + input.string() + "': " + e.what();
    return std::nullopt;
  }

  io::ReadFilter filter;
  filter.topics.push_back(topic);
  reader->set_filter(filter);

  PointCloudScan scan;
  io::RawMessage raw;
  try {
    while (reader->next(raw)) {
      scan.entries.push_back({raw.timestamp_ns});
      const auto parsed = parse_pointcloud2(raw.payload);
      if (!parsed.ok()) {
        error = parsed.error;
        return std::nullopt;
      }
      if (!accumulate_property_ranges(*parsed.cloud, scan.ranges, error)) {
        return std::nullopt;
      }
    }
  } catch (const std::exception & e) {
    error = "error reading point-cloud topic '" + topic + "': " + e.what();
    return std::nullopt;
  }

  if (scan.entries.empty()) {
    error = "point-cloud topic '" + topic + "' has no messages";
    return std::nullopt;
  }
  return scan;
}

PointCloudFetcher::PointCloudFetcher(
  const std::filesystem::path & input, std::string topic, std::vector<PointCloudIndexEntry> entries)
: input_(input), topic_(std::move(topic)), entries_(std::move(entries))
{
}

const PointCloud2 * PointCloudFetcher::fetch(std::int64_t target_ns, std::string & error)
{
  if (entries_.empty()) {
    error = "no point-cloud messages available";
    return nullptr;
  }

  const std::size_t idx = find_nearest_index(target_ns);
  const std::int64_t target_ts = entries_[idx].timestamp_ns;

  if (cached_cloud_.has_value() && cached_timestamp_ns_ == target_ts) {
    return &*cached_cloud_;
  }

  auto cloud = load_at(target_ts, error);
  if (!cloud.has_value()) {
    return nullptr;
  }
  cached_cloud_ = std::move(*cloud);
  cached_timestamp_ns_ = target_ts;
  return &*cached_cloud_;
}

std::size_t PointCloudFetcher::find_nearest_index(std::int64_t target_ns) const
{
  auto it = std::lower_bound(
    entries_.begin(), entries_.end(), target_ns,
    [](const PointCloudIndexEntry & e, std::int64_t ns) { return e.timestamp_ns < ns; });

  if (it == entries_.begin()) {
    return 0;
  }
  if (it == entries_.end()) {
    return entries_.size() - 1;
  }

  const auto prev = it - 1;
  const std::int64_t prev_delta = target_ns - prev->timestamp_ns;
  const std::int64_t next_delta = it->timestamp_ns - target_ns;
  if (prev_delta <= next_delta) {
    return static_cast<std::size_t>(prev - entries_.begin());
  }
  return static_cast<std::size_t>(it - entries_.begin());
}

std::optional<PointCloud2> PointCloudFetcher::load_at(std::int64_t ts, std::string & error)
{
  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(input_);
  } catch (const std::exception & e) {
    error = "failed to open '" + input_.string() + "': " + e.what();
    return std::nullopt;
  }

  io::ReadFilter filter;
  filter.topics.push_back(topic_);
  filter.start_ns = ts - 1;
  filter.end_ns = ts + 1;
  reader->set_filter(filter);

  io::RawMessage raw;
  try {
    while (reader->next(raw)) {
      if (raw.timestamp_ns == ts) {
        const auto parsed = parse_pointcloud2(raw.payload);
        if (!parsed.ok()) {
          error = parsed.error;
          return std::nullopt;
        }
        return std::move(*parsed.cloud);
      }
    }
  } catch (const std::exception & e) {
    error = "error reading point-cloud topic '" + topic_ + "': " + e.what();
    return std::nullopt;
  }

  error = "point-cloud message at timestamp " + std::to_string(ts) + " not found";
  return std::nullopt;
}

}  // namespace bagwiz::core::pointcloud
