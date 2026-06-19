// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/gnss_alignment.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

// Unit test for align_gnss_to_world (the closed-form 2-D Procrustes ported from
// glim_ext's gnss_global). Construction is the inverse of the function's job: we
// synthesize GNSS points by applying a KNOWN yaw + x-y translation to the
// estimated submap origins, so mapping those GNSS points back through the
// recovered world<-GNSS transform must return the original origins. No GLIM,
// no Eigen — runs in the default build.
namespace
{
namespace slam = bagwiz::core::slam;

// Apply a 2-D rotation `theta` (rad) + translation `t` to each point's x-y,
// leaving z unchanged: this is the est -> gnss transform the function must undo.
std::vector<std::array<double, 3>> apply_2d_transform(
  const std::vector<std::array<double, 3>> & pts, double theta, double tx, double ty)
{
  const double c = std::cos(theta);
  const double s = std::sin(theta);
  std::vector<std::array<double, 3>> out;
  out.reserve(pts.size());
  for (const auto & p : pts) {
    out.push_back({c * p[0] - s * p[1] + tx, s * p[0] + c * p[1] + ty, p[2]});
  }
  return out;
}

void expect_points_near(
  const std::vector<std::array<double, 3>> & got, const std::vector<std::array<double, 3>> & want,
  double tol)
{
  ASSERT_EQ(got.size(), want.size());
  for (std::size_t i = 0; i < want.size(); ++i) {
    EXPECT_NEAR(got[i][0], want[i][0], tol) << "x @ " << i;
    EXPECT_NEAR(got[i][1], want[i][1], tol) << "y @ " << i;
    EXPECT_NEAR(got[i][2], want[i][2], tol) << "z @ " << i;
  }
}

TEST(AlignGnssToWorld, RecoversKnownYawAndTranslation)
{
  const std::vector<std::array<double, 3>> est = {
    {0.0, 0.0, 0.5}, {3.0, 0.0, 1.0}, {0.0, 4.0, 1.5}, {3.0, 4.0, 2.0}};
  // GNSS = est rotated by 0.3 rad and shifted by (10, -5) in the horizontal plane.
  const auto gnss = apply_2d_transform(est, 0.3, 10.0, -5.0);

  const auto world = slam::align_gnss_to_world(est, gnss);

  // Mapping GNSS back through the recovered world<-GNSS transform returns est.
  expect_points_near(world, est, 1e-9);
}

TEST(AlignGnssToWorld, IdentityTransformReturnsInputs)
{
  const std::vector<std::array<double, 3>> est = {
    {1.0, 2.0, 0.0}, {-4.0, 1.0, 3.0}, {2.0, -3.0, 1.0}};
  // GNSS already equals est (no rotation/translation): world must equal est.
  const auto world = slam::align_gnss_to_world(est, est);
  expect_points_near(world, est, 1e-9);
}

TEST(AlignGnssToWorld, SinglePairFitsTranslationOnly)
{
  // One pair cannot determine rotation; the function leaves rotation at identity
  // and fits only the translation, so the single point maps back to est.
  const std::vector<std::array<double, 3>> est = {{2.0, 3.0, 1.0}};
  const std::vector<std::array<double, 3>> gnss = {{12.0, -7.0, 4.0}};
  const auto world = slam::align_gnss_to_world(est, gnss);
  expect_points_near(world, est, 1e-9);
}

TEST(AlignGnssToWorld, RejectsEmptyOrMismatchedInputs)
{
  EXPECT_TRUE(slam::align_gnss_to_world({}, {}).empty());

  const std::vector<std::array<double, 3>> one = {{0.0, 0.0, 0.0}};
  const std::vector<std::array<double, 3>> two = {{0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}};
  EXPECT_TRUE(slam::align_gnss_to_world(one, two).empty());
}

}  // namespace
