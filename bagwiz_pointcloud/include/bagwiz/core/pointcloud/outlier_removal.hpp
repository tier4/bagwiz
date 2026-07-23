// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__POINTCLOUD__OUTLIER_REMOVAL_HPP_
#define BAGWIZ__CORE__POINTCLOUD__OUTLIER_REMOVAL_HPP_

#include "bagwiz/core/pointcloud/kdtree.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace bagwiz::core::pointcloud
{

// Radius outlier classification: writes keep[i] = 1 when points[i] has at
// least `min_neighbors` OTHER points (itself excluded) within `radius`
// meters, else keep[i] = 0. Returns the number of outliers (keep == 0 slots
// written). min_neighbors <= 0 keeps every point.
//
// `tree` must be built over `points`. Each slot is computed independently
// from the immutable inputs, so the result is deterministic and independent
// of the thread count.
//
// No-ops returning 0: radius <= 0, or keep.size() != points.size() (nothing
// is written in either case).
std::size_t mark_radius_outliers(
  std::span<const std::array<float, 3>> points, const KdTree & tree, double radius,
  int min_neighbors, std::span<std::uint8_t> keep, int num_threads);

}  // namespace bagwiz::core::pointcloud

#endif  // BAGWIZ__CORE__POINTCLOUD__OUTLIER_REMOVAL_HPP_
