// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/concat_sync.hpp"

#include <algorithm>
#include <cstdlib>
#include <utility>
#include <vector>

namespace bagwiz::core::pointcloud
{

namespace
{

// The index of the message on `topic` whose matching stamp (stamp + offset) is
// nearest `match_target`, if within `tolerance_ns`; else nullopt. Ties resolve
// to the earliest stamp. stamps_ns is ascending, so matching stamps are too.
std::optional<std::size_t> nearest_within(
  const SyncTopic & topic, std::int64_t match_target, std::int64_t tolerance_ns)
{
  const auto & stamps = topic.stamps_ns;
  if (stamps.empty()) {
    return std::nullopt;
  }
  // A message's matching stamp is stamps[j] + offset; nearest matching stamp to
  // match_target == nearest stamp to (match_target - offset).
  const std::int64_t target_raw = match_target - topic.offset_ns;
  const auto it = std::lower_bound(stamps.begin(), stamps.end(), target_raw);

  std::optional<std::size_t> best;
  std::int64_t best_dist = 0;
  const auto consider = [&](std::size_t idx) {
    const std::int64_t match = stamps[idx] + topic.offset_ns;
    const std::int64_t dist = std::llabs(match - match_target);
    // Strict `<` keeps the earlier candidate on a tie, since the lower index
    // (earlier stamp) is considered first below.
    if (!best.has_value() || dist < best_dist) {
      best = idx;
      best_dist = dist;
    }
  };

  if (it != stamps.begin()) {
    consider(static_cast<std::size_t>((it - 1) - stamps.begin()));
  }
  if (it != stamps.end()) {
    consider(static_cast<std::size_t>(it - stamps.begin()));
  }

  if (best.has_value() && best_dist <= tolerance_ns) {
    return best;
  }
  return std::nullopt;
}

}  // namespace

std::vector<SyncGroup> plan_sync(
  const std::vector<SyncTopic> & topics, std::size_t reference, std::int64_t tolerance_ns)
{
  std::vector<SyncGroup> groups;
  if (reference >= topics.size()) {
    return groups;
  }

  const SyncTopic & ref = topics[reference];
  groups.reserve(ref.stamps_ns.size());
  for (std::size_t i = 0; i < ref.stamps_ns.size(); ++i) {
    SyncGroup group;
    group.output_stamp_ns = ref.stamps_ns[i];
    group.picks.assign(topics.size(), std::nullopt);
    group.picks[reference] = i;

    const std::int64_t match_target = ref.stamps_ns[i] + ref.offset_ns;
    for (std::size_t t = 0; t < topics.size(); ++t) {
      if (t == reference) {
        continue;
      }
      group.picks[t] = nearest_within(topics[t], match_target, tolerance_ns);
    }
    groups.push_back(std::move(group));
  }
  return groups;
}

}  // namespace bagwiz::core::pointcloud
