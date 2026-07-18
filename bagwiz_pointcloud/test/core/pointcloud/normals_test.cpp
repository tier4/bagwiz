// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/normals.hpp"

#include "bagwiz/core/pointcloud/kdtree.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

namespace
{

namespace pointcloud = bagwiz::core::pointcloud;

std::vector<std::array<float, 3>> make_grid(std::size_t side, float step, float offset_x)
{
  std::vector<std::array<float, 3>> points;
  points.reserve(side * side);
  for (std::size_t i = 0; i < side; ++i) {
    for (std::size_t j = 0; j < side; ++j) {
      points.push_back(
        {offset_x + static_cast<float>(i) * step, static_cast<float>(j) * step, 0.0f});
    }
  }
  return points;
}

}  // namespace

TEST(Normals, PlaneGridNormalsAreAxisAligned)
{
  // Defaults are k_neighbors = 12, num_threads = 1.
  const auto points = make_grid(21, 0.1f, 0.0f);
  const pointcloud::KdTree tree(points);
  const auto geometry = pointcloud::estimate_local_geometry(points, tree);
  ASSERT_EQ(geometry.normals.size(), points.size());
  ASSERT_EQ(geometry.spacings.size(), points.size());
  for (const auto & normal : geometry.normals) {
    // The sign is arbitrary; only the axis matters.
    EXPECT_NEAR(std::abs(normal[2]), 1.0f, 1e-6f);
    EXPECT_NEAR(normal[0], 0.0f, 1e-6f);
    EXPECT_NEAR(normal[1], 0.0f, 1e-6f);
  }
}

TEST(Normals, PlaneGridSpacingMatchesStep)
{
  const float step = 0.1f;
  const auto points = make_grid(21, step, 0.0f);
  const pointcloud::KdTree tree(points);
  const auto geometry = pointcloud::estimate_local_geometry(points, tree, 12, 1);
  // The 12 nearest neighbors of an interior grid point are 4 at distance s, 4
  // at s*sqrt(2) and 4 at 2s.
  const double expected = static_cast<double>(step) * (4.0 + 4.0 * std::sqrt(2.0) + 8.0) / 12.0;
  for (std::size_t i = 3; i < 18; ++i) {
    for (std::size_t j = 3; j < 18; ++j) {
      EXPECT_NEAR(geometry.spacings[i * 21 + j], expected, 1e-3);
    }
  }
}

TEST(Normals, SparseCloudYieldsNoNormal)
{
  // Each point — including the isolated one — has fewer than 3 neighbors.
  const std::vector<std::array<float, 3>> points{
    {0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {1000.0f, 1000.0f, 1000.0f}};
  const pointcloud::KdTree tree(points);
  const auto geometry = pointcloud::estimate_local_geometry(points, tree, 12, 1);
  for (const auto & normal : geometry.normals) {
    EXPECT_EQ(normal, (std::array<float, 3>{0.0f, 0.0f, 0.0f}));
  }
  // Spacings are still reported from the available neighbors.
  EXPECT_GT(geometry.spacings[2], 0.0f);
}

TEST(Normals, DegenerateCoincidentPointsYieldNoNormal)
{
  const std::vector<std::array<float, 3>> points(6, {1.0f, 2.0f, 3.0f});
  const pointcloud::KdTree tree(points);
  const auto geometry = pointcloud::estimate_local_geometry(points, tree, 12, 1);
  for (const auto & normal : geometry.normals) {
    EXPECT_EQ(normal, (std::array<float, 3>{0.0f, 0.0f, 0.0f}));
  }
  for (const float spacing : geometry.spacings) {
    EXPECT_EQ(spacing, 0.0f);
  }
}

TEST(Normals, SpacingReflectsDensity)
{
  // A sparse patch far from a dense one; each point sees only its own patch.
  auto points = make_grid(9, 0.1f, 0.0f);
  const auto sparse = make_grid(9, 0.5f, 100.0f);
  const std::size_t dense_count = points.size();
  points.insert(points.end(), sparse.begin(), sparse.end());
  const pointcloud::KdTree tree(points);
  const auto geometry = pointcloud::estimate_local_geometry(points, tree, 12, 1);
  const std::size_t dense_center = 4 * 9 + 4;
  const std::size_t sparse_center = dense_count + 4 * 9 + 4;
  EXPECT_LT(geometry.spacings[dense_center] * 2.0f, geometry.spacings[sparse_center]);
}

TEST(Normals, ThreadCountIndependent)
{
  std::mt19937 rng(5);
  std::uniform_real_distribution<float> dist(-10.0f, 10.0f);
  std::vector<std::array<float, 3>> points(500);
  for (auto & p : points) {
    p = {dist(rng), dist(rng), dist(rng)};
  }
  const pointcloud::KdTree tree(points);
  const auto serial = pointcloud::estimate_local_geometry(points, tree, 12, 1);
  const auto parallel = pointcloud::estimate_local_geometry(points, tree, 12, 4);
  EXPECT_EQ(serial.normals, parallel.normals);
  EXPECT_EQ(serial.spacings, parallel.spacings);
}
