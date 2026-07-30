// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tf/tf_forest_check.hpp"

#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace bagwiz::core
{

std::optional<std::string> validate_tf_forest(
  const std::set<std::pair<std::string, std::string>> & edges, const std::string & context)
{
  // std::ostringstream rather than fmt: bagwiz_tf carries no fmt dependency
  // (see tf_transform_format.cpp).
  for (const auto & pr : edges) {
    if (pr.first == pr.second) {
      std::ostringstream oss;
      oss << "TF tree " << context << ": self-referential edge '" << pr.first << "' -> '"
          << pr.second << "' is not allowed.";
      return oss.str();
    }
  }

  for (const auto & pr : edges) {
    // The set is ordered, so only visiting the lexicographically smaller half of
    // each pair reports a swapped pair once rather than twice.
    if (pr.first >= pr.second) {
      continue;
    }
    if (edges.count({pr.second, pr.first}) != 0) {
      std::ostringstream oss;
      oss << "TF tree " << context << ": opposite edges '" << pr.first << "' -> '" << pr.second
          << "' and '" << pr.second << "' -> '" << pr.first << "' cannot both appear.";
      return oss.str();
    }
  }

  std::unordered_map<std::string, std::string> child_to_parent;
  for (const auto & pr : edges) {
    auto ins = child_to_parent.emplace(pr.second, pr.first);
    if (!ins.second && ins.first->second != pr.first) {
      std::ostringstream oss;
      oss << "TF tree " << context << ": child frame '" << pr.second << "' has parent '"
          << ins.first->second << "' in one transform and '" << pr.first << "' in another.";
      return oss.str();
    }
  }

  std::unordered_set<std::string> all_nodes;
  for (const auto & pr : edges) {
    all_nodes.insert(pr.first);
    all_nodes.insert(pr.second);
  }

  // Every child has at most one parent by now, so walking up from each node
  // either reaches a root or revisits a frame, which is exactly a cycle.
  for (const auto & start : all_nodes) {
    std::unordered_set<std::string> seen_on_path;
    std::string cur = start;
    for (;;) {
      auto pit = child_to_parent.find(cur);
      if (pit == child_to_parent.end()) {
        break;
      }
      cur = pit->second;
      if (!seen_on_path.insert(cur).second) {
        std::ostringstream oss;
        oss << "TF tree " << context << ": edges contain a directed cycle (revisited frame '" << cur
            << "').";
        return oss.str();
      }
    }
  }

  return std::nullopt;
}

}  // namespace bagwiz::core
