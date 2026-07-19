// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/colorize_weight.hpp"

#include "bagwiz/core/image/gradient.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace
{
namespace slam = bagwiz::core::slam;
namespace image = bagwiz::core::image;

slam::VisiblePoint make_visible_point(std::uint32_t index, double u, double v, float depth)
{
  slam::VisiblePoint vp;
  vp.index = index;
  vp.u = u;
  vp.v = v;
  vp.depth = depth;
  return vp;
}

// Packed BGR24 raster of one gray level (no gradient anywhere).
std::vector<std::byte> make_flat_raster(
  std::uint32_t width, std::uint32_t height, std::uint8_t level)
{
  return std::vector<std::byte>(
    static_cast<std::size_t>(width) * 3U * height, static_cast<std::byte>(level));
}

// Packed BGR24 raster with a vertical step edge: the columns left of step_u
// are black, the rest white.
std::vector<std::byte> make_step_raster(
  std::uint32_t width, std::uint32_t height, std::uint32_t step_u)
{
  std::vector<std::byte> raster(static_cast<std::size_t>(width) * 3U * height, std::byte{0});
  for (std::uint32_t v = 0; v < height; ++v) {
    for (std::uint32_t u = step_u; u < width; ++u) {
      const std::size_t base = (static_cast<std::size_t>(v) * width + u) * 3U;
      raster[base + 0] = raster[base + 1] = raster[base + 2] = static_cast<std::byte>(255);
    }
  }
  return raster;
}

TEST(ObservationDistanceWeight, SaturatesAtTheReferenceDepth)
{
  EXPECT_DOUBLE_EQ(slam::observation_distance_weight(15.0F, 15.0), 1.0);
  // Nearer than the reference: ref / z > 1 clamps to 1.
  EXPECT_DOUBLE_EQ(slam::observation_distance_weight(10.0F, 15.0), 1.0);
}

TEST(ObservationDistanceWeight, FallsOffWithTheInverseSquare)
{
  EXPECT_DOUBLE_EQ(slam::observation_distance_weight(30.0F, 15.0), 0.25);
  EXPECT_DOUBLE_EQ(slam::observation_distance_weight(60.0F, 15.0), 0.0625);
}

TEST(ObservationDistanceWeight, ZeroDepthSaturatesToOne)
{
  // ref / 0 = inf, and inf clamps to 1; the rasterizer never emits a zero
  // depth, but the formula's edge behavior is pinned.
  EXPECT_DOUBLE_EQ(slam::observation_distance_weight(0.0F, 15.0), 1.0);
}

TEST(ObservationIncidenceWeight, HeadOnNormalGivesOne)
{
  const std::array<float, 3> normal{0.0F, 0.0F, -1.0F};  // facing the camera
  const std::array<float, 3> point{0.0F, 0.0F, 5.0F};
  const std::array<double, 3> cam_center{0.0, 0.0, 0.0};
  EXPECT_DOUBLE_EQ(slam::observation_incidence_weight(normal, point, cam_center), 1.0);
}

TEST(ObservationIncidenceWeight, GrazingNormalGivesZero)
{
  const std::array<float, 3> normal{1.0F, 0.0F, 0.0F};  // perpendicular to the view ray
  const std::array<float, 3> point{0.0F, 0.0F, 5.0F};
  const std::array<double, 3> cam_center{0.0, 0.0, 0.0};
  EXPECT_DOUBLE_EQ(slam::observation_incidence_weight(normal, point, cam_center), 0.0);
}

TEST(ObservationIncidenceWeight, NormalizesAnUnnormalizedNormal)
{
  // |n| = 5 (a 3-4-5 triangle): the cosine against the (0, 0, 5) view ray is
  // still exact, 15 / (5 * 5) = 0.6.
  const std::array<float, 3> normal{0.0F, -4.0F, -3.0F};
  const std::array<float, 3> point{0.0F, 0.0F, 5.0F};
  const std::array<double, 3> cam_center{0.0, 0.0, 0.0};
  EXPECT_DOUBLE_EQ(slam::observation_incidence_weight(normal, point, cam_center), 0.6);
}

TEST(ObservationIncidenceWeight, UsesTheViewDirectionFromTheCameraCenter)
{
  // View direction (3, 4, 0), |d| = 5: the cosine against the +x normal is
  // 3 / 5 = 0.6.
  const std::array<float, 3> normal{1.0F, 0.0F, 0.0F};
  const std::array<float, 3> point{4.0F, 6.0F, 3.0F};
  const std::array<double, 3> cam_center{1.0, 2.0, 3.0};
  EXPECT_DOUBLE_EQ(slam::observation_incidence_weight(normal, point, cam_center), 0.6);
}

TEST(ObservationIncidenceWeight, ZeroNormalCarriesNoInformation)
{
  const std::array<float, 3> normal{0.0F, 0.0F, 0.0F};
  const std::array<float, 3> point{0.0F, 0.0F, 5.0F};
  const std::array<double, 3> cam_center{0.0, 0.0, 0.0};
  EXPECT_DOUBLE_EQ(slam::observation_incidence_weight(normal, point, cam_center), 1.0);
}

TEST(ObservationIncidenceWeight, PointAtTheCameraCenterGivesOne)
{
  const std::array<float, 3> normal{0.0F, 0.0F, -1.0F};
  const std::array<float, 3> point{1.0F, 2.0F, 3.0F};
  const std::array<double, 3> cam_center{1.0, 2.0, 3.0};  // zero-length view direction
  EXPECT_DOUBLE_EQ(slam::observation_incidence_weight(normal, point, cam_center), 1.0);
}

TEST(ObservationSharpnessWeight, FlatImageHasNoGradient)
{
  const auto raster = make_flat_raster(32, 32, 100);
  EXPECT_DOUBLE_EQ(slam::observation_sharpness_weight(raster, 32, 32, 16.0, 16.0, 10.0), 0.0);
}

TEST(ObservationSharpnessWeight, DisabledGivesOne)
{
  const auto raster = make_step_raster(32, 32, 16);
  EXPECT_DOUBLE_EQ(slam::observation_sharpness_weight(raster, 32, 32, 16.0, 16.0, 0.0), 1.0);
  EXPECT_DOUBLE_EQ(slam::observation_sharpness_weight(raster, 32, 32, 16.0, 16.0, -5.0), 1.0);
}

TEST(ObservationSharpnessWeight, StepEdgeFollowsTheSaturationCurve)
{
  const auto raster = make_step_raster(32, 32, 16);
  const double g = image::sobel_gradient_magnitude_bilinear(raster, 32, 32, 16.0, 16.0);
  EXPECT_GT(g, 1000.0);  // a full 0 -> 255 step edge gives 4 * 255 = 1020
  const double w = slam::observation_sharpness_weight(raster, 32, 32, 16.0, 16.0, 10.0);
  EXPECT_DOUBLE_EQ(w, g / (g + 10.0));
  EXPECT_GT(w, 0.98);
}

TEST(ObservationBorderWeight, ImageCenterSaturates)
{
  EXPECT_DOUBLE_EQ(slam::observation_border_weight(50.0, 50.0, 101, 101, 16.0), 1.0);
}

TEST(ObservationBorderWeight, CornerIsZero)
{
  EXPECT_DOUBLE_EQ(slam::observation_border_weight(0.0, 0.0, 101, 101, 16.0), 0.0);
}

TEST(ObservationBorderWeight, RampsLinearlyInsideTheMargin)
{
  EXPECT_DOUBLE_EQ(slam::observation_border_weight(8.0, 50.0, 101, 101, 16.0), 0.5);
  EXPECT_DOUBLE_EQ(slam::observation_border_weight(4.0, 7.0, 101, 101, 16.0), 0.25);
}

TEST(ObservationBorderWeight, TakesTheNearestBorder)
{
  // 100 - 96 = 4 px from the right border; 100 - 95 = 5 px from the bottom.
  EXPECT_DOUBLE_EQ(slam::observation_border_weight(96.0, 50.0, 101, 101, 16.0), 0.25);
  EXPECT_DOUBLE_EQ(slam::observation_border_weight(50.0, 95.0, 101, 101, 16.0), 0.3125);
}

TEST(ObservationBorderWeight, AcceptsSubpixelCoordinates)
{
  EXPECT_DOUBLE_EQ(slam::observation_border_weight(2.5, 50.0, 101, 101, 16.0), 0.15625);
}

TEST(ObservationBorderWeight, DisabledGivesOne)
{
  EXPECT_DOUBLE_EQ(slam::observation_border_weight(0.0, 0.0, 101, 101, 0.0), 1.0);
  EXPECT_DOUBLE_EQ(slam::observation_border_weight(0.0, 0.0, 101, 101, -4.0), 1.0);
}

TEST(ComputeObservationWeight, DistanceOnlyWhenOtherTermsAreDisabled)
{
  const auto raster = make_flat_raster(101, 101, 200);
  const std::vector<std::array<float, 3>> points = {{0.0F, 0.0F, 5.0F}};
  const auto vp = make_visible_point(0, 50.0, 50.0, 30.0F);
  const slam::ObservationWeightParams params{15.0, 0.0, 0.0};  // sharpness/border off
  const std::array<double, 3> cam_center{0.0, 0.0, 0.0};
  EXPECT_DOUBLE_EQ(
    slam::compute_observation_weight(vp, points, {}, cam_center, raster, 101, 101, params), 0.25);
}

TEST(ComputeObservationWeight, MultipliesTheIncidenceTerm)
{
  const auto raster = make_flat_raster(101, 101, 200);
  const std::vector<std::array<float, 3>> points = {{0.0F, 0.0F, 5.0F}};
  const std::vector<std::array<float, 3>> normals = {{0.0F, -4.0F, -3.0F}};  // w_inc = 0.6
  const auto vp = make_visible_point(0, 50.0, 50.0, 30.0F);
  const slam::ObservationWeightParams params{15.0, 0.0, 0.0};
  const std::array<double, 3> cam_center{0.0, 0.0, 0.0};
  EXPECT_DOUBLE_EQ(
    slam::compute_observation_weight(vp, points, normals, cam_center, raster, 101, 101, params),
    0.15);
}

TEST(ComputeObservationWeight, OutOfRangeNormalIndexFallsBackToOne)
{
  const auto raster = make_flat_raster(101, 101, 200);
  const std::vector<std::array<float, 3>> points = {
    {0.0F, 0.0F, 5.0F}, {1.0F, 0.0F, 5.0F}, {2.0F, 0.0F, 5.0F}, {3.0F, 0.0F, 5.0F}};
  const std::vector<std::array<float, 3>> normals = {{0.0F, -4.0F, -3.0F}};  // no entry for index 3
  const auto vp = make_visible_point(3, 50.0, 50.0, 30.0F);
  const slam::ObservationWeightParams params{15.0, 0.0, 0.0};
  const std::array<double, 3> cam_center{0.0, 0.0, 0.0};
  EXPECT_DOUBLE_EQ(
    slam::compute_observation_weight(vp, points, normals, cam_center, raster, 101, 101, params),
    0.25);
}

TEST(ComputeObservationWeight, ZeroNormalFallsBackToOne)
{
  const auto raster = make_flat_raster(101, 101, 200);
  const std::vector<std::array<float, 3>> points = {{0.0F, 0.0F, 5.0F}};
  const std::vector<std::array<float, 3>> normals = {{0.0F, 0.0F, 0.0F}};
  const auto vp = make_visible_point(0, 50.0, 50.0, 30.0F);
  const slam::ObservationWeightParams params{15.0, 0.0, 0.0};
  const std::array<double, 3> cam_center{0.0, 0.0, 0.0};
  EXPECT_DOUBLE_EQ(
    slam::compute_observation_weight(vp, points, normals, cam_center, raster, 101, 101, params),
    0.25);
}

TEST(ComputeObservationWeight, MultipliesTheBorderTerm)
{
  const auto raster = make_flat_raster(101, 101, 200);
  const std::vector<std::array<float, 3>> points = {{0.0F, 0.0F, 5.0F}};
  const auto vp = make_visible_point(0, 8.0, 50.0, 30.0F);  // 8 px in -> w_border = 0.5
  const slam::ObservationWeightParams params{15.0, 0.0, 16.0};
  const std::array<double, 3> cam_center{0.0, 0.0, 0.0};
  EXPECT_DOUBLE_EQ(
    slam::compute_observation_weight(vp, points, {}, cam_center, raster, 101, 101, params), 0.125);
}

TEST(ComputeObservationWeight, MultipliesTheSharpnessTerm)
{
  const auto raster = make_step_raster(101, 101, 50);
  const std::vector<std::array<float, 3>> points = {{0.0F, 0.0F, 5.0F}};
  const auto vp = make_visible_point(0, 50.0, 50.0, 30.0F);     // on the step edge
  const slam::ObservationWeightParams params{15.0, 10.0, 0.0};  // border off
  const std::array<double, 3> cam_center{0.0, 0.0, 0.0};
  const double g = image::sobel_gradient_magnitude_bilinear(raster, 101, 101, 50.0, 50.0);
  EXPECT_GT(g, 0.0);
  EXPECT_DOUBLE_EQ(
    slam::compute_observation_weight(vp, points, {}, cam_center, raster, 101, 101, params),
    0.25 * (g / (g + 10.0)));
}

TEST(ComputeObservationWeight, MultipliesAllFourTerms)
{
  // The step edge sits at column 8, so the sampled point has both an active
  // sharpness term (edge column) and an active border term (8 px in -> 0.5).
  const auto raster = make_step_raster(101, 101, 8);
  const std::vector<std::array<float, 3>> points = {{0.0F, 0.0F, 5.0F}};
  const std::vector<std::array<float, 3>> normals = {{0.0F, -4.0F, -3.0F}};  // w_inc = 0.6
  const auto vp = make_visible_point(0, 8.0, 50.0, 30.0F);
  const slam::ObservationWeightParams params{15.0, 10.0, 16.0};
  const std::array<double, 3> cam_center{0.0, 0.0, 0.0};
  const double g = image::sobel_gradient_magnitude_bilinear(raster, 101, 101, 8.0, 50.0);
  EXPECT_GT(g, 0.0);
  // ((w_dist * w_inc) * w_sharp) * w_border = ((0.25 * 0.6) * g/(g+10)) * 0.5
  EXPECT_DOUBLE_EQ(
    slam::compute_observation_weight(vp, points, normals, cam_center, raster, 101, 101, params),
    0.25 * 0.6 * (g / (g + 10.0)) * 0.5);
}

}  // namespace
