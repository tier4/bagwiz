// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/gnss_alignment.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
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

// --- gnss_targets_with_offset: lever-arm-aware variant ---------------------

// An L-shaped trajectory: the first leg heads +x (heading 0), then the vehicle
// turns and heads +y (heading 90 deg). A body-fixed antenna offset therefore
// points in DIFFERENT world directions on the two legs, which is exactly the
// heading-dependent error a single rigid alignment cannot absorb.
const std::vector<std::array<double, 3>> kLOrigins = {
  {0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {20.0, 0.0, 0.0}, {20.0, 10.0, 0.0}, {20.0, 20.0, 0.0}};
// Per-submap heading (rad): three submaps facing +x, two facing +y.
const std::vector<double> kLHeadings = {0.0, 0.0, 0.0, M_PI / 2, M_PI / 2};

// Rotate a body-frame lever arm into the world frame for each heading (2-D yaw;
// z carried through). This is what cloud_mapper computes as R_world_origin·lever.
std::vector<std::array<double, 3>> offsets_for(
  const std::vector<double> & headings, double lx, double ly, double lz)
{
  std::vector<std::array<double, 3>> out;
  out.reserve(headings.size());
  for (const double h : headings) {
    const double c = std::cos(h);
    const double s = std::sin(h);
    out.push_back({c * lx - s * ly, s * lx + c * ly, lz});
  }
  return out;
}

std::vector<std::array<double, 3>> add_points(
  const std::vector<std::array<double, 3>> & a, const std::vector<std::array<double, 3>> & b)
{
  std::vector<std::array<double, 3>> out;
  out.reserve(a.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    out.push_back({a[i][0] + b[i][0], a[i][1] + b[i][1], a[i][2] + b[i][2]});
  }
  return out;
}

double max_horizontal_dev(
  const std::vector<std::array<double, 3>> & a, const std::vector<std::array<double, 3>> & b)
{
  double m = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    m = std::max(m, std::hypot(a[i][0] - b[i][0], a[i][1] - b[i][1]));
  }
  return m;
}

TEST(GnssTargetsWithOffset, ZeroOffsetMatchesPlainAlignment)
{
  // With no lever arm the variant must reproduce align_gnss_to_world exactly.
  const std::vector<std::array<double, 3>> est = {
    {0.0, 0.0, 0.5}, {3.0, 0.0, 1.0}, {0.0, 4.0, 1.5}, {3.0, 4.0, 2.0}};
  const auto gnss = apply_2d_transform(est, 0.3, 10.0, -5.0);
  const std::vector<std::array<double, 3>> zero_offsets(est.size(), {0.0, 0.0, 0.0});

  const auto plain = slam::align_gnss_to_world(est, gnss);
  const auto with_offset = slam::gnss_targets_with_offset(est, zero_offsets, gnss);

  expect_points_near(with_offset.targets, plain, 1e-12);
}

TEST(GnssTargetsWithOffset, RemovesHeadingDependentLeverArm)
{
  // Antenna is 1.5 m ahead of the sensor (body +x). The GNSS reports the antenna
  // position: origin + R_world_origin·lever. The corrected targets must map back
  // onto the submap origins; ignoring the lever arm cannot.
  const auto offsets = offsets_for(kLHeadings, 1.5, 0.0, 0.0);
  const auto gnss = add_points(kLOrigins, offsets);  // perfect antenna fixes, identity datum

  const auto result = slam::gnss_targets_with_offset(kLOrigins, offsets, gnss);
  expect_points_near(result.targets, kLOrigins, 1e-9);

  // Contrast: the plain alignment (no lever-arm awareness) feeds the antenna
  // positions straight in, so its targets retain a heading-dependent bias well
  // above a few cm — the very error this variant removes.
  const auto biased = slam::align_gnss_to_world(kLOrigins, gnss);
  EXPECT_GT(max_horizontal_dev(biased, kLOrigins), 0.3)
    << "test setup is degenerate: plain alignment already matched the origins";
}

TEST(GnssTargetsWithOffset, RecoversThroughDatumYawAndTranslation)
{
  // Same lever arm, but the GNSS datum is rotated 0.4 rad and shifted (100, -50):
  // the antenna-to-antenna fit must still recover, and removing the offset must
  // still land the targets back on the submap origins. The reported ENU->world
  // rotation must recover that same datum yaw (cos 0.4, sin 0.4).
  const auto offsets = offsets_for(kLHeadings, 1.5, 0.5, 0.0);
  const auto antenna = add_points(kLOrigins, offsets);
  const auto gnss = apply_2d_transform(antenna, 0.4, 100.0, -50.0);

  const auto result = slam::gnss_targets_with_offset(kLOrigins, offsets, gnss);
  expect_points_near(result.targets, kLOrigins, 1e-9);
  EXPECT_NEAR(result.world_from_enu_cos, std::cos(0.4), 1e-9);
  EXPECT_NEAR(result.world_from_enu_sin, std::sin(0.4), 1e-9);
}

TEST(GnssTargetsWithOffset, RejectsEmptyOrMismatchedInputs)
{
  EXPECT_TRUE(slam::gnss_targets_with_offset({}, {}, {}).targets.empty());

  const std::vector<std::array<double, 3>> two = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
  const std::vector<std::array<double, 3>> one_off = {{0.0, 0.0, 0.0}};
  // offsets length mismatched with origins/gnss -> empty.
  EXPECT_TRUE(slam::gnss_targets_with_offset(two, one_off, two).targets.empty());
}

// --- gnss_world_prior_covariance: ENU->world rotation + floor + inflation -----

constexpr double kUnconstrainedZVar = 1e8;  // ~ (1e4 m)^2: z effectively free

TEST(GnssWorldPriorCovariance, IdentityRotationPassesHorizontalThrough)
{
  // cos=1, sin=0, no floor, no inflation: the horizontal block is unchanged and z
  // is whatever z_variance we ask for.
  const std::array<double, 4> cov = {0.5, 0.1, 0.1, 0.9};
  const auto w = slam::gnss_world_prior_covariance(cov, 1.0, 0.0, 0.0, 1.0, kUnconstrainedZVar);
  EXPECT_NEAR(w[0], 0.5, 1e-12);
  EXPECT_NEAR(w[1], 0.1, 1e-12);
  EXPECT_NEAR(w[3], 0.1, 1e-12);
  EXPECT_NEAR(w[4], 0.9, 1e-12);
  EXPECT_EQ(w[2], 0.0);
  EXPECT_EQ(w[5], 0.0);
  EXPECT_EQ(w[6], 0.0);
  EXPECT_EQ(w[7], 0.0);
  EXPECT_DOUBLE_EQ(w[8], kUnconstrainedZVar);
}

TEST(GnssWorldPriorCovariance, NinetyDegreeRotationSwapsAxes)
{
  // A diagonal ENU covariance rotated by 90deg (cos=0, sin=1, M=[[0,1],[-1,0]])
  // must swap the East/North variances in the world frame.
  const std::array<double, 4> cov = {0.25, 0.0, 0.0, 4.0};
  const auto w = slam::gnss_world_prior_covariance(cov, 0.0, 1.0, 0.0, 1.0, kUnconstrainedZVar);
  EXPECT_NEAR(w[0], 4.0, 1e-12);   // was N
  EXPECT_NEAR(w[4], 0.25, 1e-12);  // was E
  EXPECT_NEAR(w[1], 0.0, 1e-12);
}

TEST(GnssWorldPriorCovariance, RotationPreservesTraceAndIsSymmetric)
{
  // A similarity rotation preserves the trace; the output must stay symmetric.
  const std::array<double, 4> cov = {0.58, -0.57, -0.57, 1.43};
  const double theta = 0.7;
  const auto w = slam::gnss_world_prior_covariance(
    cov, std::cos(theta), std::sin(theta), 0.0, 1.0, kUnconstrainedZVar);
  EXPECT_NEAR(w[0] + w[4], 0.58 + 1.43, 1e-9);  // trace preserved
  EXPECT_NEAR(w[1], w[3], 1e-12);               // symmetric
}

TEST(GnssWorldPriorCovariance, FloorRaisesOverOptimisticVariance)
{
  // A wildly optimistic 1 mm covariance must be lifted to at least the floor^2.
  const std::array<double, 4> cov = {1e-6, 0.0, 0.0, 1e-6};
  const double floor = 0.1;  // 10 cm
  const auto w = slam::gnss_world_prior_covariance(cov, 1.0, 0.0, floor, 1.0, kUnconstrainedZVar);
  EXPECT_GE(w[0], floor * floor - 1e-12);
  EXPECT_GE(w[4], floor * floor - 1e-12);
}

TEST(GnssWorldPriorCovariance, InflationScalesVarianceBySquare)
{
  const std::array<double, 4> cov = {0.5, 0.0, 0.0, 0.5};
  const double inflation = 2.0;
  const auto w =
    slam::gnss_world_prior_covariance(cov, 1.0, 0.0, 0.0, inflation, kUnconstrainedZVar);
  EXPECT_NEAR(w[0], 0.5 * inflation * inflation, 1e-12);  // variance scales by inflation^2
}

}  // namespace
