// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/scan_match_fill.hpp"

#include <Eigen/Geometry>

#include <gtest/gtest.h>

#include <vector>

namespace bagwiz::core::slam
{
namespace
{

// A room-corner point cloud: three axis-aligned planes meeting at the origin
// (normals along +x, +y, +z). Three orthogonal planes make the 6-DoF pose fully
// observable to GICP (a single plane would slide, two would still rotate about
// their shared edge), so registration has a unique minimum.
std::vector<Eigen::Vector3d> make_corner_cloud(double extent = 4.0, double step = 0.2)
{
  std::vector<Eigen::Vector3d> pts;
  for (double a = 0.0; a <= extent; a += step) {
    for (double b = 0.0; b <= extent; b += step) {
      pts.emplace_back(a, b, 0.0);  // z = 0 plane
      pts.emplace_back(0.0, a, b);  // x = 0 plane
      pts.emplace_back(a, 0.0, b);  // y = 0 plane
    }
  }
  return pts;
}

// Transform each world point into the LiDAR frame of a sensor at T_world_lidar:
// p_lidar = T_world_lidar^-1 * p_world. Registering this back against the world
// cloud must recover T_world_lidar.
std::vector<Eigen::Vector3d> to_lidar_frame(
  const std::vector<Eigen::Vector3d> & world, const Eigen::Isometry3d & T_world_lidar)
{
  const Eigen::Isometry3d T_lidar_world = T_world_lidar.inverse();
  std::vector<Eigen::Vector3d> out;
  out.reserve(world.size());
  for (const auto & p : world) {
    out.push_back(T_lidar_world * p);
  }
  return out;
}

Eigen::Isometry3d make_pose(const Eigen::Vector3d & t, double angle, const Eigen::Vector3d & axis)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
  pose.translation() = t;
  return pose;
}

double rotation_error(const Eigen::Isometry3d & a, const Eigen::Isometry3d & b)
{
  return Eigen::AngleAxisd(a.rotation().transpose() * b.rotation()).angle();
}

// GICP on an exact synthetic cloud converges tightly; assert well inside the
// initial perturbation so the test proves convergence, not just "didn't diverge".
constexpr double kTransTol = 0.03;  // m
constexpr double kRotTol = 0.02;    // rad

TEST(ScanMatchFill, RecoversKnownTransformFromPerturbedGuess)
{
  const auto world = make_corner_cloud();
  const Eigen::Isometry3d true_T_world_lidar = make_pose({0.7, -0.4, 0.25}, 0.12, {0.3, 0.2, 0.93});
  const auto source_lidar = to_lidar_frame(world, true_T_world_lidar);

  ScanMatchFiller filler;
  filler.insert_target(world);
  ASSERT_FALSE(filler.target_empty());

  // Start well off the true pose (0.2 m + ~0.06 rad) but inside the GICP basin.
  const Eigen::Isometry3d init =
    true_T_world_lidar * make_pose({0.2, -0.15, 0.1}, 0.06, {0.1, 0.9, 0.2});

  const ScanMatchResult result = filler.register_scan(source_lidar, init);

  EXPECT_TRUE(result.converged);
  EXPECT_GT(result.inlier_fraction, 0.9);
  EXPECT_LT(
    (result.T_world_lidar.translation() - true_T_world_lidar.translation()).norm(), kTransTol);
  EXPECT_LT(rotation_error(result.T_world_lidar, true_T_world_lidar), kRotTol);
}

// Threading each registration (ScanMatchParams.num_threads > 1 — wired from the
// run's --threads by the mapper) must not change convergence: same transform
// within the same tolerances as the single-threaded default. Guards the
// endpoint-fill acceleration on LiDAR-only runs, where the end window alone
// can hold a full odometry smoother window of scans.
TEST(ScanMatchFill, MultithreadedRegistrationMatchesSingleThreaded)
{
  const auto world = make_corner_cloud();
  const Eigen::Isometry3d true_T_world_lidar = make_pose({0.7, -0.4, 0.25}, 0.12, {0.3, 0.2, 0.93});
  const auto source_lidar = to_lidar_frame(world, true_T_world_lidar);
  const Eigen::Isometry3d init =
    true_T_world_lidar * make_pose({0.2, -0.15, 0.1}, 0.06, {0.1, 0.9, 0.2});

  ScanMatchParams params;
  params.num_threads = 4;
  ScanMatchFiller filler{params};
  filler.insert_target(world);
  ASSERT_FALSE(filler.target_empty());

  const ScanMatchResult result = filler.register_scan(source_lidar, init);

  EXPECT_TRUE(result.converged);
  EXPECT_GT(result.inlier_fraction, 0.9);
  EXPECT_LT(
    (result.T_world_lidar.translation() - true_T_world_lidar.translation()).norm(), kTransTol);
  EXPECT_LT(rotation_error(result.T_world_lidar, true_T_world_lidar), kRotTol);
}

TEST(ScanMatchFill, EmptyTargetReturnsInitGuessUnconverged)
{
  ScanMatchFiller filler;
  ASSERT_TRUE(filler.target_empty());

  const auto source = make_corner_cloud();
  const Eigen::Isometry3d init = make_pose({1.0, 2.0, 3.0}, 0.1, {0.0, 0.0, 1.0});

  const ScanMatchResult result = filler.register_scan(source, init);

  EXPECT_FALSE(result.converged);
  // The pose is left exactly at the init guess when nothing was registered.
  EXPECT_LT((result.T_world_lidar.translation() - init.translation()).norm(), 1e-12);
  EXPECT_LT(rotation_error(result.T_world_lidar, init), 1e-12);
}

TEST(ScanMatchFill, TooFewSourcePointsRejected)
{
  const auto world = make_corner_cloud();
  ScanMatchFiller filler;
  filler.insert_target(world);

  // A 3-point source is far below the 50-point floor: reject without a fit.
  const std::vector<Eigen::Vector3d> tiny_source = {
    {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}};
  const Eigen::Isometry3d init = Eigen::Isometry3d::Identity();

  const ScanMatchResult result = filler.register_scan(tiny_source, init);

  EXPECT_FALSE(result.converged);
}

// A batch below min_points must not seed the target.
TEST(ScanMatchFill, SparseTargetBatchIgnored)
{
  ScanMatchFiller filler;
  const std::vector<Eigen::Vector3d> tiny = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
  filler.insert_target(tiny);
  EXPECT_TRUE(filler.target_empty());
}

}  // namespace
}  // namespace bagwiz::core::slam
