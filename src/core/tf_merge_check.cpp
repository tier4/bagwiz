// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tf_merge_check.hpp"

#include <optional>
#include <string>

namespace bagwiz::core
{

std::optional<std::string> TfMergeConflictChecker::add(
  const std::string & parent, const std::string & child, const std::string & topic, bool is_static)
{
  Entry & entry = by_child_[child];

  // (2) Static / dynamic mix: a frame declared by both classes is contradictory
  // regardless of parent. Record this source's class, then check the other.
  if (is_static) {
    entry.static_topic = topic;
  } else {
    entry.dynamic_topic = topic;
  }
  if (!entry.static_topic.empty() && !entry.dynamic_topic.empty()) {
    return "frame '" + child + "' is declared by both a static topic ('" + entry.static_topic +
           "') and a dynamic topic ('" + entry.dynamic_topic +
           "'); a frame cannot be both static and dynamic";
  }

  // (1) Cross-topic multi-parent: anchor the first parent seen. A later edge
  // with a different parent is a conflict only when it comes from a different
  // topic — a single topic's own time series may vary without being flagged.
  if (entry.parent_topic.empty()) {
    entry.parent = parent;
    entry.parent_topic = topic;
  } else if (parent != entry.parent && topic != entry.parent_topic) {
    return "frame '" + child + "' has conflicting parents: '" + entry.parent + "' (topic '" +
           entry.parent_topic + "') and '" + parent + "' (topic '" + topic +
           "'); a frame must have a unique parent";
  }

  return std::nullopt;
}

}  // namespace bagwiz::core
