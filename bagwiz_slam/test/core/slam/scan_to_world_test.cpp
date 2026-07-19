// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/scan_to_world.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

namespace
{
namespace slam = bagwiz::core::slam;
using bagwiz::core::TrajectoryPose;

TrajectoryPose make_pose(
  std::int64_t stamp_ns, double tx = 0.0, double ty = 0.0, double tz = 0.0, double qx = 0.0,
  double qy = 0.0, double qz = 0.0, double qw = 1.0)
{
  TrajectoryPose pose;
  pose.timestamp_ns = stamp_ns;
  pose.tx = tx;
  pose.ty = ty;
  pose.tz = tz;
  pose.qx = qx;
  pose.qy = qy;
  pose.qz = qz;
  pose.qw = qw;
  return pose;
}

slam::LidarScan make_scan(std::int64_t stamp_ns, std::vector<std::array<double, 3>> points)
{
  slam::LidarScan scan;
  scan.stamp_ns = stamp_ns;
  scan.frame_id = "lidar";
  scan.points = std::move(points);
  return scan;
}

TEST(ScanToWorld, IdentityPoseKeepsPoints)
{
  const std::vector<TrajectoryPose> trajectory = {make_pose(0), make_pose(10)};
  const auto world = slam::scan_to_world_points(make_scan(5, {{1.0, 2.0, 3.0}}), trajectory);
  ASSERT_TRUE(world.has_value());
  ASSERT_EQ(world->size(), 1U);
  EXPECT_FLOAT_EQ((*world)[0][0], 1.0F);
  EXPECT_FLOAT_EQ((*world)[0][1], 2.0F);
  EXPECT_FLOAT_EQ((*world)[0][2], 3.0F);
}

TEST(ScanToWorld, TranslatesPoints)
{
  const std::vector<TrajectoryPose> trajectory = {make_pose(0, 10.0, 20.0, 30.0), make_pose(10)};
  const auto world = slam::scan_to_world_points(make_scan(0, {{1.0, 2.0, 3.0}}), trajectory);
  ASSERT_TRUE(world.has_value());
  ASSERT_EQ(world->size(), 1U);
  EXPECT_FLOAT_EQ((*world)[0][0], 11.0F);
  EXPECT_FLOAT_EQ((*world)[0][1], 22.0F);
  EXPECT_FLOAT_EQ((*world)[0][2], 33.0F);
}

TEST(ScanToWorld, RotatesPointsNinetyDegreesAboutZ)
{
  // 90 deg about z: (1, 0, 0) -> (0, 1, 0), (0, 1, 0) -> (-1, 0, 0).
  const double s = std::sqrt(0.5);
  const std::vector<TrajectoryPose> trajectory = {
    make_pose(0, 10.0, 20.0, 30.0, 0.0, 0.0, s, s), make_pose(10)};
  const auto world =
    slam::scan_to_world_points(make_scan(0, {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}}), trajectory);
  ASSERT_TRUE(world.has_value());
  ASSERT_EQ(world->size(), 2U);
  EXPECT_NEAR((*world)[0][0], 10.0F, 1e-5F);
  EXPECT_NEAR((*world)[0][1], 21.0F, 1e-5F);
  EXPECT_NEAR((*world)[0][2], 30.0F, 1e-5F);
  EXPECT_NEAR((*world)[1][0], 9.0F, 1e-5F);
  EXPECT_NEAR((*world)[1][1], 20.0F, 1e-5F);
  EXPECT_NEAR((*world)[1][2], 30.0F, 1e-5F);
}

TEST(ScanToWorld, InterpolatesThePoseBetweenPoses)
{
  const std::vector<TrajectoryPose> trajectory = {
    make_pose(0, 0.0, 0.0, 0.0), make_pose(10, 10.0, 20.0, 30.0)};
  const auto world = slam::scan_to_world_points(make_scan(5, {{1.0, 1.0, 1.0}}), trajectory);
  ASSERT_TRUE(world.has_value());
  ASSERT_EQ(world->size(), 1U);
  EXPECT_FLOAT_EQ((*world)[0][0], 6.0F);
  EXPECT_FLOAT_EQ((*world)[0][1], 11.0F);
  EXPECT_FLOAT_EQ((*world)[0][2], 16.0F);
}

TEST(ScanToWorld, EmptyTrajectoryYieldsNullopt)
{
  const std::vector<TrajectoryPose> trajectory;
  const auto world = slam::scan_to_world_points(make_scan(5, {{1.0, 2.0, 3.0}}), trajectory);
  EXPECT_FALSE(world.has_value());
}

TEST(ScanToWorld, StampBeforeTheSpanYieldsNullopt)
{
  const std::vector<TrajectoryPose> trajectory = {make_pose(0), make_pose(10)};
  const auto world = slam::scan_to_world_points(make_scan(-1, {{1.0, 2.0, 3.0}}), trajectory);
  EXPECT_FALSE(world.has_value());
}

TEST(ScanToWorld, StampAfterTheSpanYieldsNullopt)
{
  const std::vector<TrajectoryPose> trajectory = {make_pose(0), make_pose(10)};
  const auto world = slam::scan_to_world_points(make_scan(11, {{1.0, 2.0, 3.0}}), trajectory);
  EXPECT_FALSE(world.has_value());
}

TEST(ScanToWorld, SpanBoundariesAreInclusive)
{
  const std::vector<TrajectoryPose> trajectory = {make_pose(0), make_pose(10)};
  EXPECT_TRUE(slam::scan_to_world_points(make_scan(0, {{1.0, 2.0, 3.0}}), trajectory).has_value());
  EXPECT_TRUE(slam::scan_to_world_points(make_scan(10, {{1.0, 2.0, 3.0}}), trajectory).has_value());
}

TEST(ScanToWorld, EmptyScanYieldsAnEmptyCloud)
{
  const std::vector<TrajectoryPose> trajectory = {make_pose(0), make_pose(10)};
  const auto world = slam::scan_to_world_points(make_scan(5, {}), trajectory);
  ASSERT_TRUE(world.has_value());
  EXPECT_TRUE(world->empty());
}

}  // namespace
