// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__POINTCLOUD__FETCHER_HPP_
#define BAGWIZ__CORE__POINTCLOUD__FETCHER_HPP_

#include "bagwiz/core/pointcloud/pointcloud2.hpp"
#include "bagwiz/core/pointcloud/property.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace bagwiz::core::pointcloud
{

struct PointCloudIndexEntry
{
  std::int64_t timestamp_ns = 0;
};

struct PointCloudIndex
{
  std::vector<PointCloudIndexEntry> entries;
  double property_min = 0.0;
  double property_max = 0.0;
  bool has_intensity = false;
};

// Scan a PointCloud2 topic, recording every timestamp and (when no manual
// min/max is supplied) the global min/max of the selected property. Returns
// std::nullopt and fills `error` on failure.
[[nodiscard]] std::optional<PointCloudIndex> build_point_cloud_index(
  const std::filesystem::path & input, const std::string & topic, PointCloudProperty property,
  const std::optional<double> & manual_min, const std::optional<double> & manual_max,
  std::string & error);

// Per-property value extent used to drive the colour-map range. Ranges are
// indexed by PointCloudProperty (the enum values are contiguous 0..kCount-1).
// A range whose `min` exceeds `max` was never observed (e.g. intensity on a
// cloud that has no intensity field); `resolve()` substitutes a neutral [0, 1].
struct PropertyRanges
{
  static constexpr std::size_t kCount = 5;
  // Running min/max per property; default-constructed to +inf / -inf so the
  // first observed value initialises both bounds.
  std::array<double, kCount> mins;
  std::array<double, kCount> maxs;
  bool has_intensity = false;

  PropertyRanges();

  // Fold another topic's ranges into this one (element-wise min of mins / max
  // of maxs); `has_intensity` becomes true if either side had intensity.
  void merge(const PropertyRanges & other);

  // Resolved [min, max] for `property`; neutral [0, 1] when never observed.
  [[nodiscard]] std::pair<double, double> resolve(PointCloudProperty property) const;
};

// Update `running` per-property min/max from every point in `cloud`. Requires
// x/y/z fields; sets `error` and returns false when they are absent. Intensity
// is optional: when present, `running.has_intensity` becomes true and the
// intensity range is updated. Start from a default-constructed PropertyRanges
// and call once per cloud. Pure (no I/O), so it is unit-testable in isolation.
[[nodiscard]] bool accumulate_property_ranges(
  const PointCloud2 & cloud, PropertyRanges & running, std::string & error);

struct PointCloudScan
{
  std::vector<PointCloudIndexEntry> entries;
  PropertyRanges ranges;
};

// Single-pass scan of a PointCloud2 topic: records every message timestamp and
// the min/max of *all* colour properties in one read, so the interactive
// overlay can switch the active property without re-reading the bag. Returns
// std::nullopt and fills `error` on failure.
[[nodiscard]] std::optional<PointCloudScan> scan_point_cloud(
  const std::filesystem::path & input, const std::string & topic, std::string & error);

// Fetch the PointCloud2 message whose timestamp is closest to target_ns.
// The returned pointer is valid until the next fetch() call or destruction.
class PointCloudFetcher
{
public:
  PointCloudFetcher(
    const std::filesystem::path & input, std::string topic,
    std::vector<PointCloudIndexEntry> entries);

  [[nodiscard]] const PointCloud2 * fetch(std::int64_t target_ns, std::string & error);

private:
  [[nodiscard]] std::size_t find_nearest_index(std::int64_t target_ns) const;
  [[nodiscard]] std::optional<PointCloud2> load_at(std::int64_t ts, std::string & error);

  const std::filesystem::path input_;
  const std::string topic_;
  const std::vector<PointCloudIndexEntry> entries_;
  std::optional<PointCloud2> cached_cloud_;
  std::int64_t cached_timestamp_ns_ = 0;
};

}  // namespace bagwiz::core::pointcloud

#endif  // BAGWIZ__CORE__POINTCLOUD__FETCHER_HPP_
