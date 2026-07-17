// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/color_propagation.hpp"

#include <algorithm>
#include <thread>
#include <vector>

namespace bagwiz::core::pointcloud
{

std::size_t propagate_uncolored(
  std::span<const std::array<float, 3>> points, const KdTree & tree,
  std::span<std::array<std::uint8_t, 3>> colors, std::span<std::uint8_t> observed, double radius,
  int num_threads)
{
  if (colors.size() != points.size() || observed.size() != points.size() || radius <= 0.0) {
    return 0;
  }
  const std::size_t n = points.size();
  const float radius_f = static_cast<float>(radius);

  // Each chunk only writes observed == 0 slots, so per-chunk counts can simply
  // be summed after the join.
  const auto propagate_chunk = [&](std::size_t begin, std::size_t end) -> std::size_t {
    std::size_t propagated = 0;
    std::vector<std::uint32_t> candidates;
    for (std::size_t i = begin; i < end; ++i) {
      if (observed[i] != 0) {
        continue;
      }
      tree.radius_search(points[i], radius_f, candidates);
      // Nearest observed source wins; the index tie-break keeps equidistant
      // choices reproducible.
      std::uint32_t best = 0;
      float best_dist_sq = 0.0f;
      bool found = false;
      for (const std::uint32_t index : candidates) {
        if (observed[index] != 1) {
          continue;
        }
        const float dx = points[index][0] - points[i][0];
        const float dy = points[index][1] - points[i][1];
        const float dz = points[index][2] - points[i][2];
        const float dist_sq = dx * dx + dy * dy + dz * dz;
        if (!found || dist_sq < best_dist_sq || (dist_sq == best_dist_sq && index < best)) {
          found = true;
          best = index;
          best_dist_sq = dist_sq;
        }
      }
      if (found) {
        colors[i] = colors[best];
        observed[i] = 2;
        ++propagated;
      }
    }
    return propagated;
  };

  const int threads = std::max(1, num_threads);
  if (threads == 1) {
    return propagate_chunk(0, n);
  }
  std::vector<std::thread> workers;
  workers.reserve(static_cast<std::size_t>(threads));
  std::vector<std::size_t> counts(static_cast<std::size_t>(threads), 0);
  const std::size_t chunk =
    (n + static_cast<std::size_t>(threads) - 1) / static_cast<std::size_t>(threads);
  for (int t = 0; t < threads; ++t) {
    const std::size_t begin = std::min(n, static_cast<std::size_t>(t) * chunk);
    const std::size_t end = std::min(n, begin + chunk);
    workers.emplace_back([&counts, &propagate_chunk, begin, end, t] {
      counts[static_cast<std::size_t>(t)] = propagate_chunk(begin, end);
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
