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

// GLIM-free unit tests for the VoxelGrid downsampler and the Removert-style
// dynamic-point filter that backs the slam command's exported-map cleaning.
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

// --- RemovertFilter (original Removert-style dynamic-point removal) ---------

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

slam::RemovertConfig permissive_config()
{
  slam::RemovertConfig cfg;
  cfg.vertical_fov_deg = 50.0;
  cfg.horizontal_fov_deg = 360.0;
  cfg.remove_resolutions.clear();
  cfg.remove_resolutions.push_back(1.0);
  cfg.revert_resolutions.clear();
  cfg.adaptive_coeff = 0.05;
  cfg.valid_diff_upper_bound = 200.0;
  cfg.enable_revert = false;
  return cfg;
}

TEST(RemovertFilter, KeepsAStaticPoint)
{
  // Map holds one point on the wall at x=10. Every scan observes the wall there,
  // so the point is supported, never seen through.
  const std::vector<std::array<float, 3>> map = {{10.0F, 0.0F, 0.0F}};
  slam::RemovertFilter filter(permissive_config(), map);

  const auto wall = wall_at(10.0F, 2.0F, 0.1F);
  for (int i = 0; i < 5; ++i) {
    filter.add_scan({0.0, 0.0, 0.0}, wall);
  }

  const auto keep = filter.filter();
  ASSERT_EQ(keep.size(), 1U);
  EXPECT_EQ(keep[0], 1);
  EXPECT_EQ(filter.removed_count(), 0U);
}

TEST(RemovertFilter, RemovesAGhostPoint)
{
  // Map holds a "ghost" point floating at x=5. Every scan now observes the
  // background wall at x=10 in the same direction, i.e. it sees straight
  // through x=5 -> dynamic.
  const std::vector<std::array<float, 3>> map = {{5.0F, 0.0F, 0.0F}};
  slam::RemovertFilter filter(permissive_config(), map);

  const auto wall = wall_at(10.0F, 2.0F, 0.1F);
  for (int i = 0; i < 5; ++i) {
    filter.add_scan({0.0, 0.0, 0.0}, wall);
  }

  const auto keep = filter.filter();
  ASSERT_EQ(keep.size(), 1U);
  EXPECT_EQ(keep[0], 0);
  EXPECT_EQ(filter.removed_count(), 1U);
}

TEST(RemovertFilter, RemovesAnOccludedPoint)
{
  // A point BEHIND the wall (x=20, wall at x=10): every scan sees a closer
  // surface, so the point is occluded. Upstream Removert treats this as
  // a lack of visible support and removes it.
  const std::vector<std::array<float, 3>> map = {{20.0F, 0.0F, 0.0F}};
  slam::RemovertFilter filter(permissive_config(), map);

  const auto wall = wall_at(10.0F, 2.0F, 0.1F);
  for (int i = 0; i < 5; ++i) {
    filter.add_scan({0.0, 0.0, 0.0}, wall);
  }

  const auto keep = filter.filter();
  ASSERT_EQ(keep.size(), 1U);
  EXPECT_EQ(keep[0], 0);
  EXPECT_EQ(filter.removed_count(), 1U);
}

TEST(RemovertFilter, EmptyMapProducesEmptyMask)
{
  slam::RemovertFilter filter(permissive_config(), {});
  filter.add_scan({0.0, 0.0, 0.0}, wall_at(10.0F, 1.0F, 0.2F));
  const auto keep = filter.filter();
  EXPECT_TRUE(keep.empty());
}

TEST(RemovertFilter, ConsensusRevertRecoversFineFalseNegative)
{
  // Map point on the x-axis at x == 10. A fine remove resolution splits the
  // point's bin from a nearby foreground surface, leaving only a background
  // return in the point's bin, so it is seen-through and removed. A coarser
  // revert resolution merges the foreground surface into the same bin, giving
  // surface support and allowing revert.
  const std::vector<std::array<float, 3>> map = {{10.0F, 0.0F, 0.0F}};

  slam::RemovertConfig cfg;
  cfg.vertical_fov_deg = 50.0;
  cfg.horizontal_fov_deg = 360.0;
  cfg.remove_resolutions.clear();
  cfg.remove_resolutions.push_back(2.0);  // 0.5 deg/pixel azimuth
  cfg.revert_resolutions.clear();
  cfg.revert_resolutions.push_back(1.0);  // 1.0 deg/pixel azimuth
  cfg.adaptive_coeff = 0.05;
  cfg.valid_diff_upper_bound = 200.0;
  cfg.enable_revert = true;

  slam::RemovertFilter filter(cfg, map);

  // Background return shares the point's fine azimuth bin (around 0 deg).
  // Foreground return is offset by ~0.3 deg, just enough to fall into the next
  // fine bin but into the same coarse bin as the point.
  const std::vector<std::array<float, 3>> scan = {
    {20.0F, 0.0F, 0.0F},  // background, az ~0 deg, range 20
    {10.0F, 0.05F, 0.0F}  // foreground, az ~0.3 deg, range ~10
  };
  for (int i = 0; i < 3; ++i) {
    filter.add_scan({0.0, 0.0, 0.0}, scan);
  }

  const auto keep = filter.filter();
  ASSERT_EQ(keep.size(), 1U);
  EXPECT_EQ(keep[0], 1);
  EXPECT_EQ(filter.reverted_count(), 1U);
}

TEST(RemovertFilter, ConsensusRevertDoesNotRecoverTrueDynamic)
{
  // A point floating in free space in front of a wall is seen-through at every
  // resolution and must not be reverted.
  const std::vector<std::array<float, 3>> map = {{5.0F, 0.0F, 0.0F}};

  slam::RemovertConfig cfg;
  cfg.vertical_fov_deg = 50.0;
  cfg.horizontal_fov_deg = 360.0;
  cfg.remove_resolutions.assign({1.0});
  cfg.revert_resolutions.assign({0.7, 0.5});
  cfg.adaptive_coeff = 0.05;
  cfg.valid_diff_upper_bound = 200.0;
  cfg.enable_revert = true;

  slam::RemovertFilter filter(cfg, map);

  const auto wall = wall_at(10.0F, 2.0F, 0.1F);
  for (int i = 0; i < 5; ++i) {
    filter.add_scan({0.0, 0.0, 0.0}, wall);
  }

  const auto keep = filter.filter();
  ASSERT_EQ(keep.size(), 1U);
  EXPECT_EQ(keep[0], 0);
  EXPECT_EQ(filter.reverted_count(), 0U);
}

TEST(RemovertFilter, SequentialRemoveTakesUnionAcrossResolutions)
{
  // Map point on the x-axis at x == 10. At a fine resolution the foreground
  // return and background return fall into different azimuth bins, so the point
  // is supported and would be kept. At a coarser resolution they fall into the
  // same bin, the point is seen-through, and the union remove pass must still
  // drop it.
  const std::vector<std::array<float, 3>> map = {{10.0F, 0.0F, 0.0F}};

  slam::RemovertConfig cfg;
  cfg.vertical_fov_deg = 50.0;
  cfg.horizontal_fov_deg = 360.0;
  cfg.remove_resolutions.assign({2.0, 1.0});  // fine first, then coarse
  cfg.revert_resolutions.clear();
  cfg.adaptive_coeff = 0.05;
  cfg.valid_diff_upper_bound = 200.0;
  cfg.enable_revert = false;

  slam::RemovertFilter filter(cfg, map);

  const std::vector<std::array<float, 3>> scan = {
    {20.0F, 0.0F, 0.0F},  // background, az ~0 deg
    {10.0F, 0.05F, 0.0F}  // foreground, az ~0.3 deg
  };
  for (int i = 0; i < 3; ++i) {
    filter.add_scan({0.0, 0.0, 0.0}, scan);
  }

  const auto keep = filter.filter();
  ASSERT_EQ(keep.size(), 1U);
  EXPECT_EQ(keep[0], 0);
  EXPECT_EQ(filter.removed_count(), 1U);
}

}  // namespace
