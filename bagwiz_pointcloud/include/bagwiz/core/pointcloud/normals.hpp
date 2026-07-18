// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__POINTCLOUD__NORMALS_HPP_
#define BAGWIZ__CORE__POINTCLOUD__NORMALS_HPP_

#include "bagwiz/core/pointcloud/kdtree.hpp"

#include <array>
#include <span>
#include <vector>

namespace bagwiz::core::pointcloud
{

// Per-point local surface estimates, one entry per input point.
struct LocalGeometry
{
  // Unit normal from neighborhood PCA. {0, 0, 0} is the "no normal" sentinel for
  // points with too few neighbors or a degenerate (coincident) neighborhood.
  // The sign is arbitrary — consumers must use |dot(normal, v)|, never the
  // signed value.
  std::vector<std::array<float, 3>> normals;
  // Mean distance to the k nearest neighbors; a local point-density measure.
  std::vector<float> spacings;
};

// Estimates a normal and a local spacing for every point, each from a single
// kNN query against `tree` (which must be built over `points`). The normal is
// the eigenvector of the smallest eigenvalue of the neighborhood covariance
// (the query point plus its k nearest neighbors). Points with fewer than 3
// neighbors, or whose neighborhood has near-zero total variance, get the
// {0, 0, 0} "no normal" sentinel.
//
// Work is split into independent point chunks over `num_threads` std::threads
// (num_threads <= 1 runs serially); the result is identical for any thread
// count.
[[nodiscard]] LocalGeometry estimate_local_geometry(
  std::span<const std::array<float, 3>> points, const KdTree & tree, int k_neighbors = 12,
  int num_threads = 1);

}  // namespace bagwiz::core::pointcloud

#endif  // BAGWIZ__CORE__POINTCLOUD__NORMALS_HPP_
