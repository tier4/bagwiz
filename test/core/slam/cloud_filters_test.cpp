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

// --- VisibilityFilter (Removert-style dynamic-point removal) ----------------

// Build a dense "wall" of scan returns on the plane x == wall_x, spanning a
// y/z patch, so a range image has a return in (nearly) every direction bin the
// test cares about. Points are world-frame; the scan origin is the sensor.
std::vector<std::array<float, 3>> wall_at(float wall_x, float half_extent, float step)
{
  std::vector<std::array<float, 3>> pts;
  for (float y = -half_extent; y <= half_extent; y += step) {
    for (float z = -half_extent; z <= half_extent; z += step) {
      pts.push_back({wall_x, y, z});
    }
  }
  return pts;
}

slam::VisibilityFilterConfig permissive_config()
{
  slam::VisibilityFilterConfig cfg;
  // Coarse bins so the wall's discrete returns reliably populate the direction
  // bin of each map point under test (no empty-bin misses).
  cfg.azimuth_resolution_deg = 5.0;
  cfg.elevation_resolution_deg = 5.0;
  cfg.range_margin = 0.3;
  cfg.min_range = 0.5;
  cfg.max_range = 50.0;
  cfg.min_observations = 1;
  cfg.dynamic_ratio = 0.3;
  return cfg;
}

TEST(VisibilityFilter, KeepsAStaticPointObservedAtItsOwnRange)
{
  // Map holds one point on the wall at x=10. Every scan observes the wall there,
  // so the point is supported, never seen through.
  const std::vector<std::array<float, 3>> map = {{10.0F, 0.0F, 0.0F}};
  slam::VisibilityFilter filter(permissive_config(), map);

  const auto wall = wall_at(10.0F, 2.0F, 0.1F);
  for (int i = 0; i < 5; ++i) {
    filter.add_scan({0.0, 0.0, 0.0}, wall);
  }

  const auto keep = filter.keep_mask();
  ASSERT_EQ(keep.size(), 1U);
  EXPECT_EQ(keep[0], 1);  // kept
  EXPECT_EQ(filter.removed_count(), 0U);
}

TEST(VisibilityFilter, RemovesAGhostPointTheScansSeeThrough)
{
  // Map holds a "ghost" point floating at x=5 (e.g. a person present in one
  // earlier frame). Every scan now observes the background wall at x=10 in that
  // same direction, i.e. it sees straight through x=5 -> dynamic.
  const std::vector<std::array<float, 3>> map = {{5.0F, 0.0F, 0.0F}};
  slam::VisibilityFilter filter(permissive_config(), map);

  const auto wall = wall_at(10.0F, 2.0F, 0.1F);
  for (int i = 0; i < 5; ++i) {
    filter.add_scan({0.0, 0.0, 0.0}, wall);
  }

  const auto keep = filter.keep_mask();
  ASSERT_EQ(keep.size(), 1U);
  EXPECT_EQ(keep[0], 0);  // removed
  EXPECT_EQ(filter.removed_count(), 1U);
}

TEST(VisibilityFilter, DoesNotRemoveAnOccludedPointBehindTheObservedSurface)
{
  // A point BEHIND the wall (x=20, wall at x=10): every scan sees a closer
  // surface, so the point is occluded, never judged -> kept (no false positive).
  const std::vector<std::array<float, 3>> map = {{20.0F, 0.0F, 0.0F}};
  slam::VisibilityFilter filter(permissive_config(), map);

  const auto wall = wall_at(10.0F, 2.0F, 0.1F);
  for (int i = 0; i < 5; ++i) {
    filter.add_scan({0.0, 0.0, 0.0}, wall);
  }

  EXPECT_EQ(filter.keep_mask()[0], 1);  // kept (occluded, not seen-through)
  EXPECT_EQ(filter.removed_count(), 0U);
}

TEST(VisibilityFilter, RespectsMinObservationsBeforeRemoving)
{
  const std::vector<std::array<float, 3>> map = {{5.0F, 0.0F, 0.0F}};
  slam::VisibilityFilterConfig cfg = permissive_config();
  cfg.min_observations = 3;
  slam::VisibilityFilter filter(cfg, map);

  // Only two see-through looks: below the threshold, so the point survives.
  const auto wall = wall_at(10.0F, 2.0F, 0.1F);
  filter.add_scan({0.0, 0.0, 0.0}, wall);
  filter.add_scan({0.0, 0.0, 0.0}, wall);
  EXPECT_EQ(filter.keep_mask()[0], 1);  // kept (too few observations)

  // A third look crosses min_observations and removes it.
  filter.add_scan({0.0, 0.0, 0.0}, wall);
  EXPECT_EQ(filter.keep_mask()[0], 0);  // removed
}

TEST(VisibilityFilter, MixedMapKeepsStaticAndDropsGhost)
{
  // Two map points sharing the same direction from the origin: a ghost at x=5
  // and the real wall point at x=10. The filter must drop only the ghost.
  const std::vector<std::array<float, 3>> map = {{5.0F, 0.0F, 0.0F}, {10.0F, 0.0F, 0.0F}};
  slam::VisibilityFilter filter(permissive_config(), map);

  const auto wall = wall_at(10.0F, 2.0F, 0.1F);
  for (int i = 0; i < 4; ++i) {
    filter.add_scan({0.0, 0.0, 0.0}, wall);
  }

  const auto keep = filter.keep_mask();
  ASSERT_EQ(keep.size(), 2U);
  EXPECT_EQ(keep[0], 0);  // ghost removed
  EXPECT_EQ(keep[1], 1);  // wall kept
  EXPECT_EQ(filter.removed_count(), 1U);
}

TEST(VisibilityFilter, EmptyMapProducesEmptyMask)
{
  slam::VisibilityFilter filter(permissive_config(), {});
  filter.add_scan({0.0, 0.0, 0.0}, wall_at(10.0F, 1.0F, 0.2F));
  EXPECT_TRUE(filter.keep_mask().empty());
  EXPECT_EQ(filter.removed_count(), 0U);
}

}  // namespace
