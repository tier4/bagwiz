// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__TF__TF_MERGE_CHECK_HPP_
#define BAGWIZ__CORE__TF__TF_MERGE_CHECK_HPP_

#include <optional>
#include <string>
#include <unordered_map>

namespace bagwiz::core
{

// Detects conflicts while several TF topics (static + dynamic) are merged into
// one buffer or edge set — by `bagwiz traj`, `tf tree`, and `tf static calc`.
// Fed one `(parent, child)` edge at a time as the
// bag is streamed; `add` returns a human-readable description the first time an
// edge would conflict with what was already registered, or std::nullopt
// otherwise.
//
// Detection is cross-topic only — repeated edges from a single topic (a normal
// dynamic `/tf` time series) never conflict. Two conflict kinds are reported:
//
//   1. Multi-parent: the same `child_frame_id` is given different parent frames
//      by two different topics (a frame must have a unique parent).
//   2. Static/dynamic mix: the same `child_frame_id` is declared by both a
//      static topic and a dynamic topic (a frame cannot be both timeless and
//      time-varying).
//
// Edges sharing the same child and the same parent across topics of the same
// class are NOT a conflict (last-wins), matching the chosen policy.
class TfMergeConflictChecker
{
public:
  // Register one edge. `parent` is header.frame_id, `child` is child_frame_id,
  // `topic` is the source topic name, `is_static` marks the source class.
  // Returns a conflict description when this edge contradicts an earlier one,
  // else std::nullopt.
  std::optional<std::string> add(
    const std::string & parent, const std::string & child, const std::string & topic,
    bool is_static);

private:
  // What has been registered for a given child frame. `parent` / `parent_topic`
  // anchor the first parent seen (for cross-topic multi-parent detection);
  // `static_topic` / `dynamic_topic` record a representative source per class
  // (empty when that class has not declared this child yet).
  struct Entry
  {
    std::string parent;
    std::string parent_topic;
    std::string static_topic;
    std::string dynamic_topic;
  };

  std::unordered_map<std::string, Entry> by_child_;
};

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__TF__TF_MERGE_CHECK_HPP_
