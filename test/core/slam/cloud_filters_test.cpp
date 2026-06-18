// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/cloud_filters.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <vector>

// GLIM-free unit tests for the VoxelGrid downsampler that backs the slam
// command's exported-map density. Always compiled (no SLAM build required).
namespace
{
namespace slam = bagwiz::core::slam;

TEST(VoxelGrid, CollapsesPointsInOneVoxelToTheirCentroid)
{
  slam::VoxelGrid grid(1.0, /*with_intensity=*/false);
  // Three points inside the unit voxel [0,1)^3.
  grid.add(0.1F, 0.1F, 0.1F);
  grid.add(0.4F, 0.4F, 0.4F);
  grid.add(0.1F, 0.1F, 0.1F);

  ASSERT_EQ(grid.size(), 1U);
  const auto points = grid.points();
  ASSERT_EQ(points.size(), 1U);
  // Centroid = mean of the three points = (0.2, 0.2, 0.2).
  EXPECT_NEAR(points[0][0], 0.2F, 1e-5);
  EXPECT_NEAR(points[0][1], 0.2F, 1e-5);
  EXPECT_NEAR(points[0][2], 0.2F, 1e-5);
  // No intensity requested -> intensities() is empty.
  EXPECT_TRUE(grid.intensities().empty());
}

TEST(VoxelGrid, SeparatesPointsIntoDistinctVoxels)
{
  slam::VoxelGrid grid(1.0, false);
  grid.add(0.5F, 0.5F, 0.5F);  // voxel (0,0,0)
  grid.add(1.5F, 0.5F, 0.5F);  // voxel (1,0,0)
  grid.add(0.5F, 2.5F, 0.5F);  // voxel (0,2,0)

  EXPECT_EQ(grid.size(), 3U);
  EXPECT_EQ(grid.points().size(), 3U);
}

TEST(VoxelGrid, UsesFloorSoVoxelsAreContinuousAcrossZero)
{
  slam::VoxelGrid grid(1.0, false);
  // -0.5 floors to voxel index -1; +0.5 floors to 0: distinct voxels, each of
  // width 1.0 (truncation would merge them into a 2-wide voxel around zero).
  grid.add(-0.5F, 0.0F, 0.0F);
  grid.add(0.5F, 0.0F, 0.0F);

  EXPECT_EQ(grid.size(), 2U);
}

TEST(VoxelGrid, AveragesIntensityPerVoxelWhenEnabled)
{
  slam::VoxelGrid grid(1.0, /*with_intensity=*/true);
  grid.add(0.1F, 0.1F, 0.1F, 10.0F);
  grid.add(0.2F, 0.2F, 0.2F, 30.0F);

  ASSERT_EQ(grid.size(), 1U);
  const auto intensities = grid.intensities();
  ASSERT_EQ(intensities.size(), 1U);
  EXPECT_NEAR(intensities[0], 20.0F, 1e-5);  // mean(10, 30)
}

TEST(VoxelGrid, OutputOrderFollowsFirstSeenVoxel)
{
  slam::VoxelGrid grid(1.0, false);
  grid.add(5.5F, 0.0F, 0.0F);  // first-seen voxel (5,0,0)
  grid.add(0.5F, 0.0F, 0.0F);  // then voxel (0,0,0)
  grid.add(5.5F, 0.0F, 0.0F);  // back into the first voxel

  const auto points = grid.points();
  ASSERT_EQ(points.size(), 2U);
  // Deterministic: the voxel seen first is emitted first.
  EXPECT_NEAR(points[0][0], 5.5F, 1e-5);
  EXPECT_NEAR(points[1][0], 0.5F, 1e-5);
}

TEST(VoxelGrid, FinerResolutionYieldsAtLeastAsManyVoxels)
{
  const std::vector<std::array<float, 3>> pts = {
    {0.0F, 0.0F, 0.0F}, {0.3F, 0.0F, 0.0F}, {0.7F, 0.0F, 0.0F}, {1.2F, 0.0F, 0.0F}};

  slam::VoxelGrid coarse(1.0, false);
  slam::VoxelGrid fine(0.25, false);
  for (const auto & p : pts) {
    coarse.add(p[0], p[1], p[2]);
    fine.add(p[0], p[1], p[2]);
  }
  // Coarse merges {0.0,0.3,0.7} into one voxel; fine keeps them separate.
  EXPECT_LE(coarse.size(), fine.size());
  EXPECT_EQ(coarse.size(), 2U);
  EXPECT_EQ(fine.size(), 4U);
}

TEST(VoxelGrid, EmptyGridProducesEmptyOutput)
{
  slam::VoxelGrid grid(0.5, true);
  EXPECT_EQ(grid.size(), 0U);
  EXPECT_TRUE(grid.points().empty());
  EXPECT_TRUE(grid.intensities().empty());
}

}  // namespace
