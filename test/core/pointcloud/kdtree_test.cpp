// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/kdtree.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <random>
#include <utility>
#include <vector>

namespace
{

namespace pointcloud = bagwiz::core::pointcloud;

float dist_sq(const std::array<float, 3> & a, const std::array<float, 3> & b)
{
  const float dx = a[0] - b[0];
  const float dy = a[1] - b[1];
  const float dz = a[2] - b[2];
  return dx * dx + dy * dy + dz * dz;
}

std::vector<std::array<float, 3>> make_random_cloud(std::size_t n, std::uint32_t seed)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> dist(-100.0f, 100.0f);
  std::vector<std::array<float, 3>> points(n);
  for (auto & p : points) {
    p = {dist(rng), dist(rng), dist(rng)};
  }
  return points;
}

// (index, distance_sq) pairs, nearest first with the index tie-break — exactly
// the order KdTree::knn promises.
std::vector<std::pair<std::uint32_t, float>> brute_knn(
  const std::vector<std::array<float, 3>> & points, const std::array<float, 3> & query,
  std::size_t k)
{
  std::vector<std::pair<std::uint32_t, float>> all;
  all.reserve(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    all.emplace_back(static_cast<std::uint32_t>(i), dist_sq(query, points[i]));
  }
  std::sort(all.begin(), all.end(), [](const auto & a, const auto & b) {
    return a.second < b.second || (a.second == b.second && a.first < b.first);
  });
  all.resize(std::min(k, all.size()));
  return all;
}

}  // namespace

TEST(KdTree, KnnMatchesBruteForce)
{
  const auto points = make_random_cloud(1000, 42);
  const pointcloud::KdTree tree(points);
  std::mt19937 rng(7);
  std::uniform_real_distribution<float> dist(-100.0f, 100.0f);
  // Buffers stay allocated across queries, as callers are expected to reuse them.
  std::vector<std::uint32_t> indices;
  std::vector<float> distances_sq;
  for (int q = 0; q < 50; ++q) {
    const std::array<float, 3> query{dist(rng), dist(rng), dist(rng)};
    for (std::size_t k = 1; k <= 10; ++k) {
      tree.knn(query, k, indices, distances_sq);
      const auto expected = brute_knn(points, query, k);
      ASSERT_EQ(indices.size(), expected.size());
      ASSERT_EQ(distances_sq.size(), expected.size());
      for (std::size_t j = 0; j < expected.size(); ++j) {
        EXPECT_EQ(indices[j], expected[j].first);
        EXPECT_FLOAT_EQ(distances_sq[j], expected[j].second);
      }
    }
  }
}

TEST(KdTree, RadiusSearchMatchesBruteForce)
{
  const auto points = make_random_cloud(1000, 42);
  const pointcloud::KdTree tree(points);
  std::mt19937 rng(13);
  std::uniform_real_distribution<float> dist(-100.0f, 100.0f);
  std::vector<std::uint32_t> indices;
  for (const float radius : {1.0f, 10.0f, 30.0f}) {
    for (int q = 0; q < 10; ++q) {
      const std::array<float, 3> query{dist(rng), dist(rng), dist(rng)};
      tree.radius_search(query, radius, indices);
      std::vector<std::uint32_t> expected;
      for (std::size_t i = 0; i < points.size(); ++i) {
        if (dist_sq(query, points[i]) <= radius * radius) {
          expected.push_back(static_cast<std::uint32_t>(i));
        }
      }
      std::sort(indices.begin(), indices.end());
      EXPECT_EQ(indices, expected);
    }
  }
}

TEST(KdTree, EmptyTree)
{
  const pointcloud::KdTree tree;
  EXPECT_EQ(tree.size(), 0u);
  EXPECT_TRUE(tree.empty());
  std::vector<std::uint32_t> indices{1, 2, 3};
  std::vector<float> distances_sq{1.0f};
  tree.knn({0.0f, 0.0f, 0.0f}, 5, indices, distances_sq);
  EXPECT_TRUE(indices.empty());
  EXPECT_TRUE(distances_sq.empty());
  tree.radius_search({0.0f, 0.0f, 0.0f}, 10.0f, indices);
  EXPECT_TRUE(indices.empty());

  const std::vector<std::array<float, 3>> no_points;
  const pointcloud::KdTree from_empty(no_points);
  EXPECT_TRUE(from_empty.empty());
}

TEST(KdTree, SinglePoint)
{
  const std::vector<std::array<float, 3>> points{{1.0f, 2.0f, 3.0f}};
  const pointcloud::KdTree tree(points);
  EXPECT_EQ(tree.size(), 1u);
  EXPECT_FALSE(tree.empty());
  std::vector<std::uint32_t> indices;
  std::vector<float> distances_sq;
  tree.knn({1.0f, 2.0f, 3.0f}, 1, indices, distances_sq);
  ASSERT_EQ(indices.size(), 1u);
  EXPECT_EQ(indices[0], 0u);
  EXPECT_FLOAT_EQ(distances_sq[0], 0.0f);
  // k larger than the cloud is clamped to the cloud size.
  tree.knn({10.0f, 2.0f, 3.0f}, 5, indices, distances_sq);
  ASSERT_EQ(indices.size(), 1u);
  EXPECT_EQ(indices[0], 0u);
  EXPECT_FLOAT_EQ(distances_sq[0], 81.0f);
}

TEST(KdTree, DuplicatePoints)
{
  const std::vector<std::array<float, 3>> points(5, {1.0f, 2.0f, 3.0f});
  const pointcloud::KdTree tree(points);
  std::vector<std::uint32_t> indices;
  std::vector<float> distances_sq;
  tree.knn({1.0f, 2.0f, 3.0f}, 3, indices, distances_sq);
  // Equidistant points come out in index order.
  const std::vector<std::uint32_t> expected{0u, 1u, 2u};
  EXPECT_EQ(indices, expected);
  EXPECT_EQ(distances_sq, (std::vector<float>{0.0f, 0.0f, 0.0f}));
}

TEST(KdTree, ReusesOutputBuffers)
{
  const auto points = make_random_cloud(100, 1);
  const pointcloud::KdTree tree(points);
  // Pre-filled oversized buffers must be cleared and rewritten by each call.
  std::vector<std::uint32_t> indices(1000, 0xdeadbeefu);
  std::vector<float> distances_sq(1000, -1.0f);
  tree.knn({0.0f, 0.0f, 0.0f}, 4, indices, distances_sq);
  EXPECT_EQ(indices.size(), 4u);
  EXPECT_EQ(distances_sq.size(), 4u);
  const auto first = indices;
  tree.knn({50.0f, 50.0f, 50.0f}, 2, indices, distances_sq);
  EXPECT_EQ(indices.size(), 2u);
  EXPECT_EQ(distances_sq.size(), 2u);
  tree.knn({0.0f, 0.0f, 0.0f}, 4, indices, distances_sq);
  EXPECT_EQ(indices, first);
}
