// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef CORE__SLAM__VISUAL_ODOMETRY_GROUPING_HPP_
#define CORE__SLAM__VISUAL_ODOMETRY_GROUPING_HPP_

#include "bagwiz/core/slam/visual_observation.hpp"

#include <cstdint>
#include <map>
#include <span>
#include <vector>

namespace bagwiz::core::slam
{

/// Anchor-window observation grouping for multi-camera visual odometry.
///
/// Semantics:
/// - Anchors are the anchor camera's frame stamps: every observation with
///   `camera_id == anchor_camera_id` and stamp `s` opens (or joins) the group
///   `[s, s + period_ns)`. One state per group downstream.
/// - Per-camera streams must be stamp-non-decreasing across `insert` calls;
///   interleaving across cameras is arbitrary (worker threads).
/// - A non-anchor observation with stamp `t` is assignable only once the anchor
///   stream has advanced to `>= t` (anchors arrive in stamp order, so only then
///   is the covering anchor guaranteed known). Until then it is held pending.
///   Once assignable: it joins the newest known anchor window containing `t`, or
///   is DROPPED (counted) when no window covers `t` — an anchor-camera frame
///   drop. This is exposure-pattern agnostic: simultaneous triggers land near the
///   anchor stamp, LiDAR-synced staggered triggers spread across the window; both
///   are pure stamp arithmetic.
/// - A group is ready when every camera's stream head has passed its window end,
///   OR when any stream head is `max_lag_periods` periods past it (a silent
///   camera must not stall the pipeline; its late observations for an
///   already-popped group are dropped and counted).
/// - `pop_ready()`/`finish()` return groups in anchor order; `finish()` flushes
///   everything including pending observations (assign-or-drop with full
///   knowledge — no more anchors will arrive).
struct ObservationGroup
{
  std::int64_t anchor_stamp_ns = 0;
  std::vector<VisualObservation> observations;  // all cameras, stamps in [anchor, anchor+period)
};

class GroupingBuffer
{
public:
  struct Config
  {
    std::int32_t anchor_camera_id = 0;
    std::int64_t period_ns = 100'000'000;
    std::size_t camera_count = 1;
    std::int64_t max_lag_periods = 5;
  };

  explicit GroupingBuffer(Config config);

  void insert(std::span<const VisualObservation> observations);

  [[nodiscard]] std::vector<ObservationGroup> pop_ready();

  [[nodiscard]] std::vector<ObservationGroup> finish();

  [[nodiscard]] std::int64_t dropped_count() const;

private:
  Config config_;
  std::map<std::int64_t, ObservationGroup> groups_;  // keyed by anchor stamp
  std::vector<VisualObservation> pending_;           // stamp > anchor stream head
  std::vector<std::int64_t> last_stamp_;             // per camera, INT64_MIN until seen
  std::int64_t popped_until_ = INT64_MIN;            // anchors <= this already popped
  std::int64_t dropped_ = 0;

  void assign(const VisualObservation & o);
  [[nodiscard]] std::vector<ObservationGroup> take_groups_up_to(std::int64_t anchor_limit);
};

}  // namespace bagwiz::core::slam

#endif  // CORE__SLAM__VISUAL_ODOMETRY_GROUPING_HPP_
