// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/color_propagation.hpp"

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

constexpr std::array<std::uint8_t, 3> kGray{128, 128, 128};

struct Cloud
{
  std::vector<std::array<float, 3>> points;
  std::vector<std::array<std::uint8_t, 3>> colors;
  std::vector<std::uint8_t> observed;
};

}  // namespace

TEST(ColorPropagation, NeighborWithinRadiusInheritsColor)
{
  Cloud cloud{
    {{0.0f, 0.0f, 0.0f}, {0.5f, 0.0f, 0.0f}},
    {{10, 20, 30}, kGray},
    {1, 0},
  };
  const pointcloud::KdTree tree(cloud.points);
  const std::size_t propagated =
    pointcloud::propagate_uncolored(cloud.points, tree, cloud.colors, cloud.observed, 1.0, 1);
  EXPECT_EQ(propagated, 1u);
  EXPECT_EQ(cloud.colors[1], (std::array<std::uint8_t, 3>{10, 20, 30}));
  EXPECT_EQ(cloud.observed[1], 2);
  // The source is untouched.
  EXPECT_EQ(cloud.colors[0], (std::array<std::uint8_t, 3>{10, 20, 30}));
  EXPECT_EQ(cloud.observed[0], 1);
}

TEST(ColorPropagation, BeyondRadiusStaysUnobserved)
{
  Cloud cloud{
    {{0.0f, 0.0f, 0.0f}, {5.0f, 0.0f, 0.0f}},
    {{10, 20, 30}, kGray},
    {1, 0},
  };
  const pointcloud::KdTree tree(cloud.points);
  const std::size_t propagated =
    pointcloud::propagate_uncolored(cloud.points, tree, cloud.colors, cloud.observed, 1.0, 1);
  EXPECT_EQ(propagated, 0u);
  EXPECT_EQ(cloud.colors[1], kGray);
  EXPECT_EQ(cloud.observed[1], 0);
}

TEST(ColorPropagation, ObservedPointsAreUntouched)
{
  // Two observed sources within radius of each other keep their own colors, and
  // an observed == 2 slot is neither a source nor a target.
  Cloud cloud{
    {{0.0f, 0.0f, 0.0f}, {0.5f, 0.0f, 0.0f}, {10.0f, 0.0f, 0.0f}, {10.5f, 0.0f, 0.0f}},
    {{200, 0, 0}, {0, 200, 0}, {1, 2, 3}, kGray},
    {1, 1, 2, 0},
  };
  const pointcloud::KdTree tree(cloud.points);
  const std::size_t propagated =
    pointcloud::propagate_uncolored(cloud.points, tree, cloud.colors, cloud.observed, 1.0, 1);
  // Point 3 sits next to an observed == 2 point only, which is not a source.
  EXPECT_EQ(propagated, 0u);
  EXPECT_EQ(cloud.colors[0], (std::array<std::uint8_t, 3>{200, 0, 0}));
  EXPECT_EQ(cloud.colors[1], (std::array<std::uint8_t, 3>{0, 200, 0}));
  EXPECT_EQ(cloud.colors[2], (std::array<std::uint8_t, 3>{1, 2, 3}));
  EXPECT_EQ(cloud.observed[0], 1);
  EXPECT_EQ(cloud.observed[1], 1);
  EXPECT_EQ(cloud.observed[2], 2);
  EXPECT_EQ(cloud.colors[3], kGray);
  EXPECT_EQ(cloud.observed[3], 0);
}

TEST(ColorPropagation, ZeroOrNegativeRadiusIsNoOp)
{
  for (const double radius : {0.0, -1.0}) {
    Cloud cloud{
      {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}},
      {{10, 20, 30}, kGray},
      {1, 0},
    };
    const pointcloud::KdTree tree(cloud.points);
    const std::size_t propagated =
      pointcloud::propagate_uncolored(cloud.points, tree, cloud.colors, cloud.observed, radius, 1);
    EXPECT_EQ(propagated, 0u);
    EXPECT_EQ(cloud.colors[1], kGray);
    EXPECT_EQ(cloud.observed[1], 0);
  }
}

TEST(ColorPropagation, MismatchedSpanLengthsReturnZero)
{
  Cloud cloud{
    {{0.0f, 0.0f, 0.0f}, {0.5f, 0.0f, 0.0f}},
    {{10, 20, 30}, kGray},
    {1, 0},
  };
  const pointcloud::KdTree tree(cloud.points);
  const std::span<std::array<std::uint8_t, 3>> short_colors(cloud.colors.data(), 1);
  const std::size_t propagated =
    pointcloud::propagate_uncolored(cloud.points, tree, short_colors, cloud.observed, 1.0, 1);
  EXPECT_EQ(propagated, 0u);
  EXPECT_EQ(cloud.observed[1], 0);
}

TEST(ColorPropagation, NearestSourceWins)
{
  Cloud cloud{
    {{0.0f, 0.0f, 0.0f}, {0.3f, 0.0f, 0.0f}, {-0.6f, 0.0f, 0.0f}},
    {kGray, {255, 0, 0}, {0, 0, 255}},
    {0, 1, 1},
  };
  const pointcloud::KdTree tree(cloud.points);
  const std::size_t propagated =
    pointcloud::propagate_uncolored(cloud.points, tree, cloud.colors, cloud.observed, 1.0, 1);
  EXPECT_EQ(propagated, 1u);
  EXPECT_EQ(cloud.colors[0], (std::array<std::uint8_t, 3>{255, 0, 0}));
  EXPECT_EQ(cloud.observed[0], 2);
}

TEST(ColorPropagation, DoesNotCascadeWithinOnePass)
{
  // Point 1 is filled from point 0, but point 2 — in range of point 1 only —
  // is not filled in the same pass.
  Cloud cloud{
    {{0.0f, 0.0f, 0.0f}, {0.5f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}},
    {{10, 20, 30}, kGray, kGray},
    {1, 0, 0},
  };
  const pointcloud::KdTree tree(cloud.points);
  const std::size_t propagated =
    pointcloud::propagate_uncolored(cloud.points, tree, cloud.colors, cloud.observed, 0.75, 1);
  EXPECT_EQ(propagated, 1u);
  EXPECT_EQ(cloud.observed[1], 2);
  EXPECT_EQ(cloud.colors[2], kGray);
  EXPECT_EQ(cloud.observed[2], 0);
}

TEST(ColorPropagation, ThreadCountIndependent)
{
  // Exact equality is legitimate here per AGENTS.md "Numerical
  // Reproducibility": only points that were already observed act as color
  // sources, so a worker's writes never feed another worker's reads, and ties
  // break on point index.
  std::mt19937 rng(9);
  std::uniform_real_distribution<float> pos(0.0f, 10.0f);
  std::uniform_int_distribution<int> channel(0, 255);
  std::uniform_int_distribution<int> coin(0, 2);
  Cloud cloud;
  for (int i = 0; i < 2000; ++i) {
    cloud.points.push_back({pos(rng), pos(rng), pos(rng)});
    if (coin(rng) == 0) {
      cloud.colors.push_back(
        {static_cast<std::uint8_t>(channel(rng)), static_cast<std::uint8_t>(channel(rng)),
         static_cast<std::uint8_t>(channel(rng))});
      cloud.observed.push_back(1);
    } else {
      cloud.colors.push_back(kGray);
      cloud.observed.push_back(0);
    }
  }
  const pointcloud::KdTree tree(cloud.points);
  Cloud serial = cloud;
  Cloud parallel = cloud;
  const std::size_t serial_count =
    pointcloud::propagate_uncolored(serial.points, tree, serial.colors, serial.observed, 0.5, 1);
  const std::size_t parallel_count = pointcloud::propagate_uncolored(
    parallel.points, tree, parallel.colors, parallel.observed, 0.5, 4);
  EXPECT_GT(serial_count, 0u);
  EXPECT_EQ(serial_count, parallel_count);
  EXPECT_EQ(serial.colors, parallel.colors);
  EXPECT_EQ(serial.observed, parallel.observed);
}
