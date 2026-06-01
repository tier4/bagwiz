// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tf_chain.hpp"

#include <tf2/buffer_core.hpp>
#include <tf2/time.hpp>

#include <algorithm>
#include <cstddef>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace bagwiz::core
{

std::vector<std::string> resolve_chain(
  const tf2::BufferCore & buffer, const std::string & from_frame, const std::string & to_frame,
  tf2::TimePoint time)
{
  constexpr std::size_t kMaxDepth = 1024;

  auto walk_to_root = [&](const std::string & start) {
    std::vector<std::string> path;
    path.push_back(start);
    std::string cur = start;
    while (path.size() < kMaxDepth) {
      std::string parent;
      if (!buffer._getParent(cur, time, parent) || parent.empty()) {
        break;
      }
      path.push_back(parent);
      cur = parent;
    }
    return path;
  };

  const auto path_to = walk_to_root(to_frame);

  // Fast path: from_frame is on path_to (i.e. from is an ancestor of to).
  using DiffT = std::vector<std::string>::difference_type;
  for (std::size_t i = 0; i < path_to.size(); ++i) {
    if (path_to[i] == from_frame) {
      std::vector<std::string> chain(path_to.begin(), path_to.begin() + static_cast<DiffT>(i + 1));
      std::reverse(chain.begin(), chain.end());
      return chain;
    }
  }

  // General case: locate the lowest common ancestor.
  const auto path_from = walk_to_root(from_frame);
  std::unordered_set<std::string> to_ancestors(path_to.begin(), path_to.end());
  std::size_t lca_in_from = path_from.size();
  for (std::size_t i = 0; i < path_from.size(); ++i) {
    if (to_ancestors.count(path_from[i]) != 0) {
      lca_in_from = i;
      break;
    }
  }
  if (lca_in_from == path_from.size()) {
    return {};
  }
  const std::string & lca = path_from[lca_in_from];

  std::vector<std::string> chain(
    path_from.begin(), path_from.begin() + static_cast<DiffT>(lca_in_from + 1));
  std::size_t lca_in_to = 0;
  for (; lca_in_to < path_to.size(); ++lca_in_to) {
    if (path_to[lca_in_to] == lca) {
      break;
    }
  }
  for (std::size_t i = lca_in_to; i-- > 0;) {
    chain.push_back(path_to[i]);
  }
  return chain;
}

std::vector<std::pair<std::string, std::string>> chain_to_edges(
  const tf2::BufferCore & buffer, const std::vector<std::string> & chain, tf2::TimePoint time)
{
  std::vector<std::pair<std::string, std::string>> edges;
  if (chain.size() < 2) {
    return edges;
  }
  edges.reserve(chain.size() - 1);
  for (std::size_t i = 0; i + 1 < chain.size(); ++i) {
    const auto & a = chain[i];
    const auto & b = chain[i + 1];
    std::string parent_of_a;
    if (buffer._getParent(a, time, parent_of_a) && parent_of_a == b) {
      // b is parent of a — TF edge is published as (parent=b, child=a)
      edges.emplace_back(b, a);
    } else {
      // a is parent of b — TF edge is (parent=a, child=b)
      edges.emplace_back(a, b);
    }
  }
  return edges;
}

}  // namespace bagwiz::core
