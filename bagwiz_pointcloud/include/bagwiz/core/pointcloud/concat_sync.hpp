// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__POINTCLOUD__CONCAT_SYNC_HPP_
#define BAGWIZ__CORE__POINTCLOUD__CONCAT_SYNC_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace bagwiz::core::pointcloud
{

// One input topic's message timeline for concat matching: ascending REAL header
// stamps, plus this topic's --stamp-offset. The matching time of message i is
// `stamps_ns[i] + offset_ns` (the offset is a matching aid only — it never
// changes the real stamp used for output or per-point time).
struct SyncTopic
{
  std::vector<std::int64_t> stamps_ns;  // ascending real header.stamp values
  std::int64_t offset_ns = 0;           // --stamp-offset for this topic
};

// One output group: the reference message plus, per input topic (in the same
// order as the `topics` argument), the index of the message chosen for it —
// or std::nullopt when no message fell within tolerance (partial emit).
struct SyncGroup
{
  std::int64_t output_stamp_ns = 0;               // reference message's REAL stamp
  std::vector<std::optional<std::size_t>> picks;  // one per topic, input order
};

// Reference-driven nearest-match plan. `topics` are in --pcd order and
// `reference` indexes the reference topic. For each reference message, every
// other topic contributes the message whose matching stamp is nearest to the
// reference matching stamp within `tolerance_ns` (ties resolve to the earliest
// stamp — deterministic). The reference's own pick is always populated; missing
// topics stay std::nullopt. One SyncGroup is produced per reference message,
// in reference order.
[[nodiscard]] std::vector<SyncGroup> plan_sync(
  const std::vector<SyncTopic> & topics, std::size_t reference, std::int64_t tolerance_ns);

}  // namespace bagwiz::core::pointcloud

#endif  // BAGWIZ__CORE__POINTCLOUD__CONCAT_SYNC_HPP_
