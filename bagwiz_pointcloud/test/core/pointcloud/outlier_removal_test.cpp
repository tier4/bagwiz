// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/outlier_removal.hpp"

#include "bagwiz/core/pointcloud/kdtree.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <random>
#include <span>
#include <vector>

namespace
{

namespace pointcloud = bagwiz::core::pointcloud;

// Brute-force reference: count of OTHER points within radius.
std::size_t neighbors_within(
  const std::vector<std::array<float, 3>> & points, std::size_t i, double radius)
{
  std::size_t count = 0;
  for (std::size_t j = 0; j < points.size(); ++j) {
    if (j == i) {
      continue;
    }
    const double dx = points[j][0] - points[i][0];
    const double dy = points[j][1] - points[i][1];
    const double dz = points[j][2] - points[i][2];
    if (dx * dx + dy * dy + dz * dz <= radius * radius) {
      ++count;
    }
  }
  return count;
}

}  // namespace

TEST(OutlierRemoval, IsolatedPointIsMarkedOutlier)
{
  // A tight 4-point cluster at the origin plus one point far away.
  std::vector<std::array<float, 3>> points{
    {0.0f, 0.0f, 0.0f}, {0.1f, 0.0f, 0.0f},    {0.0f, 0.1f, 0.0f},
    {0.0f, 0.0f, 0.1f}, {10.0f, 10.0f, 10.0f},
  };
  const pointcloud::KdTree tree(points);
  std::vector<std::uint8_t> keep(points.size(), 0);
  const std::size_t removed = pointcloud::mark_radius_outliers(points, tree, 0.5, 3, keep, 1);
  EXPECT_EQ(removed, 1u);
  EXPECT_EQ(keep, (std::vector<std::uint8_t>{1, 1, 1, 1, 0}));
}

TEST(OutlierRemoval, SelfDoesNotCountAsNeighbor)
{
  // Each point of a pair has exactly ONE other point in range: min_neighbors 1
  // keeps both, min_neighbors 2 drops both. Pins the self-exclusion semantics.
  std::vector<std::array<float, 3>> points{{0.0f, 0.0f, 0.0f}, {0.5f, 0.0f, 0.0f}};
  const pointcloud::KdTree tree(points);
  std::vector<std::uint8_t> keep(points.size(), 0);

  EXPECT_EQ(pointcloud::mark_radius_outliers(points, tree, 1.0, 1, keep, 1), 0u);
  EXPECT_EQ(keep, (std::vector<std::uint8_t>{1, 1}));

  EXPECT_EQ(pointcloud::mark_radius_outliers(points, tree, 1.0, 2, keep, 1), 2u);
  EXPECT_EQ(keep, (std::vector<std::uint8_t>{0, 0}));
}

TEST(OutlierRemoval, DuplicatePointsCountEachOther)
{
  std::vector<std::array<float, 3>> points{{1.0f, 2.0f, 3.0f}, {1.0f, 2.0f, 3.0f}};
  const pointcloud::KdTree tree(points);
  std::vector<std::uint8_t> keep(points.size(), 0);
  const std::size_t removed = pointcloud::mark_radius_outliers(points, tree, 0.1, 1, keep, 1);
  EXPECT_EQ(removed, 0u);
  EXPECT_EQ(keep, (std::vector<std::uint8_t>{1, 1}));
}

TEST(OutlierRemoval, ZeroOrNegativeRadiusIsNoOp)
{
  std::vector<std::array<float, 3>> points{{0.0f, 0.0f, 0.0f}, {0.1f, 0.0f, 0.0f}};
  const pointcloud::KdTree tree(points);
  for (const double radius : {0.0, -1.0}) {
    std::vector<std::uint8_t> keep(points.size(), 7);
    const std::size_t removed = pointcloud::mark_radius_outliers(points, tree, radius, 1, keep, 1);
    EXPECT_EQ(removed, 0u);
    // Nothing was written.
    EXPECT_EQ(keep, (std::vector<std::uint8_t>{7, 7}));
  }
}

TEST(OutlierRemoval, MismatchedKeepSizeIsNoOp)
{
  std::vector<std::array<float, 3>> points{{0.0f, 0.0f, 0.0f}, {0.1f, 0.0f, 0.0f}};
  const pointcloud::KdTree tree(points);
  std::vector<std::uint8_t> keep(1, 7);
  const std::size_t removed = pointcloud::mark_radius_outliers(points, tree, 1.0, 1, keep, 1);
  EXPECT_EQ(removed, 0u);
  EXPECT_EQ(keep, (std::vector<std::uint8_t>{7}));
}

TEST(OutlierRemoval, NonPositiveMinNeighborsKeepsEverything)
{
  std::vector<std::array<float, 3>> points{{0.0f, 0.0f, 0.0f}, {100.0f, 0.0f, 0.0f}};
  const pointcloud::KdTree tree(points);
  std::vector<std::uint8_t> keep(points.size(), 0);
  const std::size_t removed = pointcloud::mark_radius_outliers(points, tree, 1.0, 0, keep, 1);
  EXPECT_EQ(removed, 0u);
  EXPECT_EQ(keep, (std::vector<std::uint8_t>{1, 1}));
}

TEST(OutlierRemoval, MatchesBruteForceReference)
{
  std::mt19937 rng(17);
  std::uniform_real_distribution<float> pos(0.0f, 5.0f);
  std::vector<std::array<float, 3>> points;
  for (int i = 0; i < 400; ++i) {
    points.push_back({pos(rng), pos(rng), pos(rng)});
  }
  const double radius = 0.4;
  const int min_neighbors = 3;
  const pointcloud::KdTree tree(points);
  std::vector<std::uint8_t> keep(points.size(), 0);
  const std::size_t removed =
    pointcloud::mark_radius_outliers(points, tree, radius, min_neighbors, keep, 1);

  std::size_t expected_removed = 0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    const bool expected_keep =
      neighbors_within(points, i, radius) >= static_cast<std::size_t>(min_neighbors);
    EXPECT_EQ(keep[i], expected_keep ? 1 : 0) << "point " << i;
    if (!expected_keep) {
      ++expected_removed;
    }
  }
  EXPECT_EQ(removed, expected_removed);
  // The uniform draw at this density leaves both classes non-empty.
  EXPECT_GT(removed, 0u);
  EXPECT_LT(removed, points.size());
}

TEST(OutlierRemoval, ThreadCountIndependent)
{
  std::mt19937 rng(23);
  std::uniform_real_distribution<float> pos(0.0f, 10.0f);
  std::vector<std::array<float, 3>> points;
  for (int i = 0; i < 2000; ++i) {
    points.push_back({pos(rng), pos(rng), pos(rng)});
  }
  const pointcloud::KdTree tree(points);
  std::vector<std::uint8_t> serial(points.size(), 0);
  std::vector<std::uint8_t> parallel(points.size(), 0);
  const std::size_t serial_removed =
    pointcloud::mark_radius_outliers(points, tree, 0.5, 3, serial, 1);
  const std::size_t parallel_removed =
    pointcloud::mark_radius_outliers(points, tree, 0.5, 3, parallel, 4);
  EXPECT_GT(serial_removed, 0u);
  EXPECT_EQ(serial_removed, parallel_removed);
  EXPECT_EQ(serial, parallel);
}
