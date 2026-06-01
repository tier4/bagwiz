// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__TF_WALK_TIMELINE_HPP_
#define BAGWIZ__CORE__TF_WALK_TIMELINE_HPP_

#include <tf2/buffer_core.hpp>
#include <tf2/time.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <optional>
#include <string>
#include <vector>

// Timeline + per-step resolution for `bagwiz tf walk`. The walk merges every
// TF topic of a bag into one buffer and steps through the distinct times at
// which the merged TF changed, resolving <from> -> <to> at each. These helpers
// are the pure, bag-free core (no I/O, no terminal) so they can be unit-tested
// against an in-memory tf2::BufferCore; the command layer owns the bag reading
// and the pager. The walk does not classify transforms as static vs dynamic.
namespace bagwiz::core
{

// One step of a tf walk: the query time on the merged TF timeline and the
// resolved <from> -> <to> transform at that time. `transform` is std::nullopt
// when the buffer cannot connect the two frames at `time` (e.g. the chain is
// not yet complete, or the time is outside a dynamic frame's cached window);
// `error` then carries the tf2 reason and is empty on success.
struct TfWalkStep
{
  tf2::TimePoint time;
  std::optional<geometry_msgs::msg::TransformStamped> transform;
  std::string error;
};

// Sort `stamps` ascending and drop duplicates, yielding the ordered set of
// distinct times at which the merged TF changed. The input may be unsorted and
// contain duplicates (the same stamp on several transforms / topics); an empty
// input yields an empty timeline.
std::vector<tf2::TimePoint> build_tf_walk_timeline(std::vector<tf2::TimePoint> stamps);

// Resolve <from> -> <to> from `buffer` at `time`, matching
// lookupTransform(target=to_frame, source=from_frame) — the tf2_echo
// convention, where the translation is <from>'s origin expressed in <to>. A
// tf2::TransformException is caught and reported via the step's `error`
// (transform left empty) rather than thrown, so a single unresolvable time
// does not abort the walk.
TfWalkStep resolve_tf_walk_step(
  const tf2::BufferCore & buffer, tf2::TimePoint time, const std::string & from_frame,
  const std::string & to_frame);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__TF_WALK_TIMELINE_HPP_
