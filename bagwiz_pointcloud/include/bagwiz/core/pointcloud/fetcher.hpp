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
  // Matching key: the cloud's header.stamp (sec * 1e9 + nanosec), used to find
  // the cloud nearest a camera frame's header.stamp. Falls back to record_ns
  // when the source header.stamp is unset (0). Entries are kept sorted by this.
  std::int64_t stamp_ns = 0;
  // Seek key: the bag record time. The storage layer indexes messages by record
  // time (not header.stamp), so loading a matched cloud seeks by this value.
  std::int64_t record_ns = 0;
};

struct PointCloudIndex
{
  std::vector<PointCloudIndexEntry> entries;
  double property_min = 0.0;
  double property_max = 0.0;
  bool has_intensity = false;
  // True only when *every* message carried a real header.stamp, so stamp_ns is a
  // pure capture-time axis that can be matched against a camera frame's
  // header.stamp. False if any message fell back to record time: the axis is then
  // mixed and can't be compared to either clock, so callers must match this topic
  // by record time (PointCloudMatchKey::kRecordTime) instead.
  bool header_stamps_present = false;
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
  // See PointCloudIndex::header_stamps_present.
  bool header_stamps_present = false;
};

// Single-pass scan of a PointCloud2 topic: records every message timestamp and
// the min/max of *all* colour properties in one read, so the interactive
// overlay can switch the active property without re-reading the bag. Returns
// std::nullopt and fills `error` on failure.
[[nodiscard]] std::optional<PointCloudScan> scan_point_cloud(
  const std::filesystem::path & input, const std::string & topic, std::string & error);

// Which clock a fetch matches against. A cloud topic can be matched by capture
// time only when every message carries a header.stamp (see
// PointCloudIndex::header_stamps_present); otherwise both the camera frame and
// the cloud must be matched by bag record time so the comparison stays within a
// single clock.
enum class PointCloudMatchKey { kHeaderStamp, kRecordTime };

// Fetch the PointCloud2 message whose key (header.stamp or record time) is
// closest to target_ns. The matched cloud is then loaded from storage by its bag
// record time. The returned pointer is valid until the next fetch() call or
// destruction.
class PointCloudFetcher
{
public:
  PointCloudFetcher(
    const std::filesystem::path & input, std::string topic,
    std::vector<PointCloudIndexEntry> entries);

  // Fetch the cloud whose `key` clock is closest to target_ns; target_ns must be
  // expressed in that same clock (a header.stamp for kHeaderStamp, a bag record
  // time for kRecordTime).
  [[nodiscard]] const PointCloud2 * fetch(
    std::int64_t target_ns, PointCloudMatchKey key, std::string & error);

private:
  [[nodiscard]] static std::size_t find_nearest_index(
    const std::vector<PointCloudIndexEntry> & entries, std::int64_t target_ns,
    PointCloudMatchKey key);
  [[nodiscard]] std::optional<PointCloud2> load_at(std::int64_t record_ns, std::string & error);

  const std::filesystem::path input_;
  const std::string topic_;
  // The same entries in two orders so fetch() can binary-search whichever clock
  // the caller matches in: by_stamp_ is sorted by stamp_ns, by_record_ by
  // record_ns.
  const std::vector<PointCloudIndexEntry> by_stamp_;
  const std::vector<PointCloudIndexEntry> by_record_;
  std::optional<PointCloud2> cached_cloud_;
  std::int64_t cached_record_ns_ = 0;
};

}  // namespace bagwiz::core::pointcloud

#endif  // BAGWIZ__CORE__POINTCLOUD__FETCHER_HPP_
