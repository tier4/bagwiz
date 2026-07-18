// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__POINTCLOUD__COLOR_PROPAGATION_HPP_
#define BAGWIZ__CORE__POINTCLOUD__COLOR_PROPAGATION_HPP_

#include "bagwiz/core/pointcloud/kdtree.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace bagwiz::core::pointcloud
{

// Fills every unobserved point (observed == 0) with the color of the nearest
// observed point (observed == 1) within `radius`, and marks filled points as
// observed == 2 ("propagated"). Ties between equidistant sources are broken by
// point index, so the result is reproducible. Returns the number of propagated
// points.
//
// `tree` must be built over `points`. Source slots (observed == 1) are read
// but never written, and only observed == 0 slots are written, so a point
// filled during this pass never becomes a source for another point in the same
// pass — the outcome is independent of the thread count.
//
// No-ops returning 0: radius <= 0, or any of the spans differing in length
// (nothing is written in either case).
std::size_t propagate_uncolored(
  std::span<const std::array<float, 3>> points, const KdTree & tree,
  std::span<std::array<std::uint8_t, 3>> colors, std::span<std::uint8_t> observed, double radius,
  int num_threads);

}  // namespace bagwiz::core::pointcloud

#endif  // BAGWIZ__CORE__POINTCLOUD__COLOR_PROPAGATION_HPP_
