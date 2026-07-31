// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "visual_odometry_grouping.hpp"  // NOLINT(build/include_subdir) src-local header

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace bagwiz::core::slam
{

GroupingBuffer::GroupingBuffer(Config config) : config_(config)
{
  last_stamp_.assign(config_.camera_count, std::numeric_limits<std::int64_t>::min());
  // Fail fast on an impossible rig description: insert() indexes
  // last_stamp_ by anchor_camera_id unguarded, so an out-of-range anchor
  // would be UB on the very first call.
  if (
    config_.camera_count == 0 || config_.anchor_camera_id < 0 ||
    static_cast<std::size_t>(config_.anchor_camera_id) >= config_.camera_count) {
    throw std::invalid_argument("GroupingBuffer: anchor_camera_id must index a camera");
  }
}

void GroupingBuffer::insert(std::span<const VisualObservation> observations)
{
  // Pass 1: anchors first, so windows opened by this batch can receive this
  // batch's own non-anchor observations.
  for (const auto & o : observations) {
    if (o.camera_id == config_.anchor_camera_id) {
      groups_[o.stamp_ns].anchor_stamp_ns = o.stamp_ns;
      groups_[o.stamp_ns].observations.push_back(o);
    }
    if (
      o.camera_id >= 0 && static_cast<std::size_t>(o.camera_id) < last_stamp_.size() &&
      o.stamp_ns > last_stamp_[o.camera_id]) {
      last_stamp_[o.camera_id] = o.stamp_ns;
    }
  }
  const std::int64_t anchor_head = last_stamp_[config_.anchor_camera_id];
  // Pass 2: non-anchor observations. Assignable only once the anchor stream
  // has reached the observation's stamp — anchors arrive in stamp order, so
  // only then is the covering window guaranteed known.
  for (const auto & o : observations) {
    if (o.camera_id == config_.anchor_camera_id) {
      continue;
    }
    if (o.stamp_ns <= anchor_head) {
      assign(o);
    } else {
      pending_.push_back(o);
    }
  }
  // Drain pending entries the anchor head has caught up with.
  std::vector<VisualObservation> still_pending;
  still_pending.reserve(pending_.size());
  for (const auto & o : pending_) {
    if (o.stamp_ns <= anchor_head) {
      assign(o);
    } else {
      still_pending.push_back(o);
    }
  }
  pending_ = std::move(still_pending);
}

void GroupingBuffer::assign(const VisualObservation & o)
{
  // Newest anchor window at or before the stamp; drop when it does not
  // cover it (anchor-camera frame drop) or was already popped (late camera).
  auto it = groups_.upper_bound(o.stamp_ns);
  if (it == groups_.begin()) {
    ++dropped_;
    return;
  }
  --it;
  if (o.stamp_ns >= it->first + config_.period_ns || it->first <= popped_until_) {
    ++dropped_;
    return;
  }
  it->second.observations.push_back(o);
}

std::vector<ObservationGroup> GroupingBuffer::pop_ready()
{
  std::int64_t min_head = std::numeric_limits<std::int64_t>::max();
  std::int64_t max_head = std::numeric_limits<std::int64_t>::min();
  for (const std::int64_t s : last_stamp_) {
    min_head = std::min(min_head, s);
    max_head = std::max(max_head, s);
  }
  // Ready when every camera has passed the window end, or when any camera is
  // max_lag_periods past it (a silent camera must not stall the pipeline).
  std::int64_t limit = std::numeric_limits<std::int64_t>::min();
  for (const auto & [anchor, group] : groups_) {
    const std::int64_t window_end = anchor + config_.period_ns;
    if (
      min_head >= window_end ||
      max_head >= window_end + config_.max_lag_periods * config_.period_ns) {
      limit = anchor;
    } else {
      break;  // groups_ is anchor-ordered; later windows end later
    }
  }
  return take_groups_up_to(limit);
}

std::vector<ObservationGroup> GroupingBuffer::finish()
{
  // No more anchors will arrive: resolve every pending observation with full
  // knowledge, then flush all remaining groups in order.
  for (const auto & o : pending_) {
    assign(o);
  }
  pending_.clear();
  return take_groups_up_to(std::numeric_limits<std::int64_t>::max());
}

std::vector<ObservationGroup> GroupingBuffer::take_groups_up_to(std::int64_t anchor_limit)
{
  std::vector<ObservationGroup> out;
  auto it = groups_.begin();
  while (it != groups_.end() && it->first <= anchor_limit) {
    popped_until_ = it->first;
    out.push_back(std::move(it->second));
    it = groups_.erase(it);
  }
  return out;
}

std::int64_t GroupingBuffer::dropped_count() const
{
  return dropped_;
}

}  // namespace bagwiz::core::slam
