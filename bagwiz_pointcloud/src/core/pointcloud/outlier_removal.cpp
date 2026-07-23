// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/outlier_removal.hpp"

#include <algorithm>
#include <thread>
#include <vector>

namespace bagwiz::core::pointcloud
{

std::size_t mark_radius_outliers(
  std::span<const std::array<float, 3>> points, const KdTree & tree, double radius,
  int min_neighbors, std::span<std::uint8_t> keep, int num_threads)
{
  if (keep.size() != points.size() || radius <= 0.0) {
    return 0;
  }
  const std::size_t n = points.size();
  const float radius_f = static_cast<float>(radius);
  // radius_search finds the query point itself (it is in the tree at distance
  // 0), so surviving takes min_neighbors + 1 hits.
  const std::size_t required = static_cast<std::size_t>(std::max(min_neighbors, 0)) + 1;

  // Each chunk writes its own disjoint keep[] slots, so per-chunk counts can
  // simply be summed after the join.
  const auto mark_chunk = [&](std::size_t begin, std::size_t end) -> std::size_t {
    std::size_t removed = 0;
    std::vector<std::uint32_t> neighbors;
    for (std::size_t i = begin; i < end; ++i) {
      tree.radius_search(points[i], radius_f, neighbors);
      const bool kept = neighbors.size() >= required;
      keep[i] = kept ? 1 : 0;
      if (!kept) {
        ++removed;
      }
    }
    return removed;
  };

  const int threads = std::max(1, num_threads);
  if (threads == 1) {
    return mark_chunk(0, n);
  }
  std::vector<std::thread> workers;
  workers.reserve(static_cast<std::size_t>(threads));
  std::vector<std::size_t> counts(static_cast<std::size_t>(threads), 0);
  const std::size_t chunk =
    (n + static_cast<std::size_t>(threads) - 1) / static_cast<std::size_t>(threads);
  for (int t = 0; t < threads; ++t) {
    const std::size_t begin = std::min(n, static_cast<std::size_t>(t) * chunk);
    const std::size_t end = std::min(n, begin + chunk);
    workers.emplace_back([&counts, &mark_chunk, begin, end, t] {
      counts[static_cast<std::size_t>(t)] = mark_chunk(begin, end);
    });
  }
  for (auto & w : workers) {
    w.join();
  }
  std::size_t total = 0;
  for (const std::size_t count : counts) {
    total += count;
  }
  return total;
}

}  // namespace bagwiz::core::pointcloud
