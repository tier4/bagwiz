// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/map_colorizer.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace
{
namespace slam = bagwiz::core::slam;
using bagwiz::core::TrajectoryPose;

constexpr std::array<std::uint8_t, 3> kGray{128, 128, 128};
constexpr std::array<std::uint8_t, 3> kRed{255, 0, 0};
constexpr std::array<std::uint8_t, 3> kGreen{0, 255, 0};

TrajectoryPose make_pose(std::int64_t stamp_ns, double tx = 0.0, double ty = 0.0, double tz = 0.0)
{
  TrajectoryPose pose;
  pose.timestamp_ns = stamp_ns;
  pose.tx = tx;
  pose.ty = ty;
  pose.tz = tz;
  pose.qx = 0.0;
  pose.qy = 0.0;
  pose.qz = 0.0;
  pose.qw = 1.0;
  return pose;
}

// A 100x100 pinhole with fx = fy = 100 and the principal point at the image
// center; no distortion. A point at (0, 0, z) projects to pixel (50, 50).
bagwiz::core::image::CameraInfo make_pinhole()
{
  bagwiz::core::image::CameraInfo info;
  info.width = 100;
  info.height = 100;
  info.distortion_model = "plumb_bob";
  info.k = {100.0, 0.0, 50.0, 0.0, 100.0, 50.0, 0.0, 0.0, 1.0};
  return info;
}

// Pins the legacy-approximating configuration most tests assume: uniform
// weights, no gain compensation, no splat, single-threaded rasterization.
slam::MapColorizerConfig make_config()
{
  slam::MapColorizerConfig config;
  config.camera = make_pinhole();
  config.use_weights = false;
  config.gain_compensation = false;
  config.rasterizer.splat = false;
  config.rasterizer.num_threads = 1;
  return config;
}

// Packed BGR24 raster filled with one color ({r, g, b} given in RGB order).
std::vector<std::byte> make_raster(
  std::uint32_t width, std::uint32_t height, const std::array<std::uint8_t, 3> & rgb)
{
  std::vector<std::byte> raster(static_cast<std::size_t>(width) * 3U * height);
  for (std::size_t i = 0; i < static_cast<std::size_t>(width) * height; ++i) {
    raster[i * 3 + 0] = static_cast<std::byte>(rgb[2]);
    raster[i * 3 + 1] = static_cast<std::byte>(rgb[1]);
    raster[i * 3 + 2] = static_cast<std::byte>(rgb[0]);
  }
  return raster;
}

void set_pixel(
  std::vector<std::byte> & raster, std::uint32_t width, std::uint32_t u, std::uint32_t v,
  const std::array<std::uint8_t, 3> & rgb)
{
  const std::size_t base = (static_cast<std::size_t>(v) * width + u) * 3U;
  raster[base + 0] = static_cast<std::byte>(rgb[2]);
  raster[base + 1] = static_cast<std::byte>(rgb[1]);
  raster[base + 2] = static_cast<std::byte>(rgb[0]);
}

TEST(MapColorizer, ColorsPointFromSingleImage)
{
  const std::vector<std::array<float, 3>> points = {{0.0F, 0.0F, 5.0F}};
  const std::vector<TrajectoryPose> trajectory = {make_pose(0)};
  slam::MapColorizer colorizer(make_config(), points, trajectory);

  const auto raster = make_raster(100, 100, kRed);
  EXPECT_TRUE(colorizer.add_image(0, raster, 100, 100));

  const auto result = colorizer.finish();
  ASSERT_EQ(result.colors.size(), 1U);
  EXPECT_EQ(result.colors[0], kRed);
  EXPECT_EQ(result.colored_points, 1U);
  EXPECT_EQ(result.images_used, 1U);
  EXPECT_EQ(result.images_skipped, 0U);
}

TEST(MapColorizer, SkipsImageOutsideTrajectorySpan)
{
  const std::vector<std::array<float, 3>> points = {{0.0F, 0.0F, 5.0F}};
  const std::vector<TrajectoryPose> trajectory = {make_pose(0), make_pose(1'000'000'000)};
  slam::MapColorizer colorizer(make_config(), points, trajectory);

  const auto raster = make_raster(100, 100, kRed);
  EXPECT_FALSE(colorizer.add_image(2'000'000'000, raster, 100, 100));
  EXPECT_FALSE(colorizer.add_image(-1, raster, 100, 100));

  const auto result = colorizer.finish();
  EXPECT_EQ(result.colors[0], kGray);
  EXPECT_EQ(result.colored_points, 0U);
  EXPECT_EQ(result.images_used, 0U);
  EXPECT_EQ(result.images_skipped, 2U);
}

TEST(MapColorizer, OcclusionBlocksFarPointBehindADenseWall)
{
  // A dense 5x5 wall of points at z = 5 covers the pixels around the image
  // center; the wall's center point lands exactly on pixel (50, 50). The far
  // point at z = 10 projects to the same pixel and must be rejected.
  std::vector<std::array<float, 3>> points;
  for (const float gx : {-0.1F, -0.05F, 0.0F, 0.05F, 0.1F}) {
    for (const float gy : {-0.1F, -0.05F, 0.0F, 0.05F, 0.1F}) {
      points.push_back({gx, gy, 5.0F});
    }
  }
  points.push_back({0.0F, 0.0F, 10.0F});  // far point, index 25
  const std::vector<TrajectoryPose> trajectory = {make_pose(0)};
  slam::MapColorizer colorizer(make_config(), points, trajectory);

  EXPECT_TRUE(colorizer.add_image(0, make_raster(100, 100, kRed), 100, 100));

  const auto result = colorizer.finish();
  for (std::size_t i = 0; i < 25; ++i) {
    EXPECT_EQ(result.colors[i], kRed) << "wall point " << i << " should be colored";
  }
  EXPECT_EQ(result.colors[25], kGray);
  EXPECT_EQ(result.observed[25], 0);
  EXPECT_EQ(result.colored_points, 25U);
}

TEST(MapColorizer, SplatClosesHolesBetweenSparseOccluders)
{
  // Four sparse front points at z = 5 land on the four pixels diagonally
  // around (50, 50); the far point at z = 10 projects onto (50, 50) itself.
  // With splatting on, the front points' discs cover that pixel and occlude
  // the far point; with splatting off the hole stays open and the far point
  // legitimately wins its (otherwise unwritten) pixel.
  const std::vector<std::array<float, 3>> points = {
    {-0.05F, -0.05F, 5.0F},
    {0.05F, -0.05F, 5.0F},
    {-0.05F, 0.05F, 5.0F},
    {0.05F, 0.05F, 5.0F},
    {0.0F, 0.0F, 10.0F}};
  const std::vector<TrajectoryPose> trajectory = {make_pose(0)};
  const auto raster = make_raster(100, 100, kRed);

  auto run = [&](bool splat) {
    auto config = make_config();
    config.rasterizer.splat = splat;
    slam::MapColorizer colorizer(config, points, trajectory);
    EXPECT_TRUE(colorizer.add_image(0, raster, 100, 100));
    return colorizer.finish();
  };

  const auto with_splat = run(true);
  EXPECT_EQ(with_splat.colors[4], kGray);
  EXPECT_EQ(with_splat.observed[4], 0);
  EXPECT_EQ(with_splat.colored_points, 4U);

  const auto without_splat = run(false);
  EXPECT_EQ(without_splat.colors[4], kRed);
  EXPECT_EQ(without_splat.colored_points, 5U);
}

TEST(MapColorizer, AveragesObservationsAcrossImages)
{
  const std::vector<std::array<float, 3>> points = {{0.0F, 0.0F, 5.0F}};
  const std::vector<TrajectoryPose> trajectory = {make_pose(0), make_pose(1'000'000'000)};
  slam::MapColorizer colorizer(make_config(), points, trajectory);

  EXPECT_TRUE(colorizer.add_image(0, make_raster(100, 100, {255, 10, 20}), 100, 100));
  EXPECT_TRUE(colorizer.add_image(1'000'000'000, make_raster(100, 100, {215, 30, 50}), 100, 100));

  const auto result = colorizer.finish();
  // Both observations sit within the trim band of the lit-mode anchor, so the
  // final color is their equal-weight average in linear light: each channel is
  // decoded sRGB -> linear, averaged, and re-encoded, which lands brighter
  // than the sRGB-value midpoint (e.g. 236 for the red pair, not 235).
  EXPECT_EQ(result.colors[0], (std::array<std::uint8_t, 3>{236, 22, 38}));
  EXPECT_EQ(result.images_used, 2U);
}

TEST(MapColorizer, AveragesExposuresInLinearLight)
{
  // One surface seen at two exposures a factor of ~4.6 apart in linear light
  // (sRGB 40 and 88 — a deviation of exactly 48 code values, the widest pair
  // the trim band keeps). The mean of the underlying radiance re-encodes to
  // 69; averaging the gamma-encoded values instead would give
  // (40 + 88) / 2 = 64, a systematically darker result (Jensen's inequality:
  // the sRGB encode is concave, so an sRGB-space mean always lands at or
  // below the true mean).
  const std::vector<std::array<float, 3>> points = {{0.0F, 0.0F, 5.0F}};
  const std::vector<TrajectoryPose> trajectory = {make_pose(0), make_pose(1'000'000'000)};
  slam::MapColorizer colorizer(make_config(), points, trajectory);

  EXPECT_TRUE(colorizer.add_image(0, make_raster(100, 100, {40, 40, 40}), 100, 100));
  EXPECT_TRUE(colorizer.add_image(1'000'000'000, make_raster(100, 100, {88, 88, 88}), 100, 100));

  const auto result = colorizer.finish();
  EXPECT_EQ(result.colors[0], (std::array<std::uint8_t, 3>{69, 69, 69}));
  EXPECT_EQ(result.images_used, 2U);
}

TEST(MapColorizer, TrimsAnOutlierObservation)
{
  const std::vector<std::array<float, 3>> points = {{0.0F, 0.0F, 5.0F}};
  const std::vector<TrajectoryPose> trajectory = {make_pose(0), make_pose(5)};
  slam::MapColorizer colorizer(make_config(), points, trajectory);

  const auto majority = make_raster(100, 100, {200, 50, 60});
  const auto outlier = make_raster(100, 100, {0, 255, 0});
  for (std::int64_t stamp = 0; stamp < 5; ++stamp) {
    EXPECT_TRUE(colorizer.add_image(stamp, majority, 100, 100));
  }
  EXPECT_TRUE(colorizer.add_image(5, outlier, 100, 100));

  const auto result = colorizer.finish();
  // The outlier deviates from the anchor observation far beyond the trim
  // band; the five agreeing observations decide the final color.
  EXPECT_EQ(result.colors[0], (std::array<std::uint8_t, 3>{200, 50, 60}));
  EXPECT_EQ(result.images_used, 6U);
}

TEST(MapColorizer, PrefersTheLitModeCluster)
{
  // A surface seen eight times sunlit and eight times in shadow: shadows are
  // illumination, not surface color, so the trim anchors at the 75th
  // luminance percentile and the sunlit cluster decides the final color.
  const std::vector<std::array<float, 3>> points = {{0.0F, 0.0F, 5.0F}};
  const std::vector<TrajectoryPose> trajectory = {make_pose(0), make_pose(15)};
  slam::MapColorizer colorizer(make_config(), points, trajectory);

  const auto sunlit = make_raster(100, 100, {200, 40, 60});
  const auto shadowed = make_raster(100, 100, {100, 20, 30});
  for (std::int64_t stamp = 0; stamp < 16; ++stamp) {
    EXPECT_TRUE(colorizer.add_image(stamp, stamp % 2 == 0 ? sunlit : shadowed, 100, 100));
  }

  const auto result = colorizer.finish();
  EXPECT_EQ(result.colors[0], (std::array<std::uint8_t, 3>{200, 40, 60}));
  EXPECT_EQ(result.images_used, 16U);
}

TEST(MapColorizer, DynamicOccluderBlocksAnObservation)
{
  // A map point with a clear static view, but a scan return sits 5 m closer
  // along the same ray — a vehicle that is not in the map. The dynamic test
  // rejects the observation instead of sampling the vehicle's pixels.
  const std::vector<std::array<float, 3>> points = {{0.0F, 0.0F, 10.0F}};
  const std::vector<std::array<float, 3>> dynamic = {
    {0.0F, 0.0F, 5.0F},  // the occluder, projecting to the same pixel
    {0.0F, 0.0F, -5.0F}  // behind the camera: ignored
  };
  const std::vector<TrajectoryPose> trajectory = {make_pose(0)};
  slam::MapColorizer colorizer(make_config(), points, trajectory);

  EXPECT_TRUE(colorizer.add_image(
    0, make_raster(100, 100, kRed), 100, 100, std::span<const std::array<float, 3>>(dynamic)));

  const auto result = colorizer.finish();
  EXPECT_EQ(result.observed[0], 0U);
  EXPECT_EQ(result.colors[0], kGray);
  EXPECT_EQ(result.colored_points, 0U);
}

TEST(MapColorizer, DynamicReturnOnTheSameSurfaceKeepsTheObservation)
{
  // The scan sees the very surface the point lies on: within the dynamic
  // tolerance, so the observation passes and takes the pixel color.
  const std::vector<std::array<float, 3>> points = {{0.0F, 0.0F, 10.0F}};
  const std::vector<std::array<float, 3>> dynamic = {{0.0F, 0.0F, 9.6F}};
  const std::vector<TrajectoryPose> trajectory = {make_pose(0)};
  slam::MapColorizer colorizer(make_config(), points, trajectory);

  EXPECT_TRUE(colorizer.add_image(
    0, make_raster(100, 100, kRed), 100, 100, std::span<const std::array<float, 3>>(dynamic)));

  const auto result = colorizer.finish();
  EXPECT_EQ(result.observed[0], 1U);
  EXPECT_EQ(result.colors[0], kRed);
}

TEST(MapColorizer, ScanConfirmationOverridesAStaleMapOccluder)
{
  // A dense wall of stale geometry at z = 5 (e.g. a vehicle smeared at an
  // earlier position) statically occludes the far point at z = 10 — but the
  // scan at the image's time sees the far point's own surface. Where the
  // scan covers the pixel it is the visibility oracle, so the observation is
  // accepted despite the map occluder.
  std::vector<std::array<float, 3>> points;
  for (const float gx : {-0.1F, -0.05F, 0.0F, 0.05F, 0.1F}) {
    for (const float gy : {-0.1F, -0.05F, 0.0F, 0.05F, 0.1F}) {
      points.push_back({gx, gy, 5.0F});
    }
  }
  points.push_back({0.0F, 0.0F, 10.0F});  // far point, index 25
  const std::vector<std::array<float, 3>> dynamic = {{0.0F, 0.0F, 9.8F}};
  const std::vector<TrajectoryPose> trajectory = {make_pose(0)};
  slam::MapColorizer colorizer(make_config(), points, trajectory);

  EXPECT_TRUE(colorizer.add_image(
    0, make_raster(100, 100, kRed), 100, 100, std::span<const std::array<float, 3>>(dynamic)));

  const auto result = colorizer.finish();
  EXPECT_EQ(result.observed[25], 1U);
  EXPECT_EQ(result.colors[25], kRed);
}

TEST(MapColorizer, CullsPointsBeyondMaxRange)
{
  const std::vector<std::array<float, 3>> points = {{0.0F, 0.0F, 5.0F}};
  const std::vector<TrajectoryPose> trajectory = {make_pose(0)};
  auto config = make_config();
  config.rasterizer.max_range = 3.0;
  slam::MapColorizer colorizer(config, points, trajectory);

  EXPECT_TRUE(colorizer.add_image(0, make_raster(100, 100, kRed), 100, 100));

  const auto result = colorizer.finish();
  EXPECT_EQ(result.colors[0], kGray);
  EXPECT_EQ(result.colored_points, 0U);
}

TEST(MapColorizer, IgnoresPointsBehindTheCamera)
{
  const std::vector<std::array<float, 3>> points = {{0.0F, 0.0F, -5.0F}};
  const std::vector<TrajectoryPose> trajectory = {make_pose(0)};
  slam::MapColorizer colorizer(make_config(), points, trajectory);

  EXPECT_TRUE(colorizer.add_image(0, make_raster(100, 100, kRed), 100, 100));

  const auto result = colorizer.finish();
  EXPECT_EQ(result.colors[0], kGray);
  EXPECT_EQ(result.images_used, 1U);
}

TEST(MapColorizer, SamplesTheProjectedPixel)
{
  // x/z = 0.2 -> u = 100 * 0.2 + 50 = 70, v = 50.
  const std::vector<std::array<float, 3>> points = {{1.0F, 0.0F, 5.0F}};
  const std::vector<TrajectoryPose> trajectory = {make_pose(0)};
  slam::MapColorizer colorizer(make_config(), points, trajectory);

  auto raster = make_raster(100, 100, {0, 0, 0});
  set_pixel(raster, 100, 70, 50, kGreen);
  EXPECT_TRUE(colorizer.add_image(0, raster, 100, 100));

  const auto result = colorizer.finish();
  EXPECT_EQ(result.colors[0], kGreen);
}

TEST(MapColorizer, BilinearlySamplesBetweenPixels)
{
  // x/z = 0.005 -> u = 50.5, exactly on the boundary between the black pixel
  // column 50 and the white column 51: the bilinear sample blends both.
  const std::vector<std::array<float, 3>> points = {{0.025F, 0.0F, 5.0F}};
  const std::vector<TrajectoryPose> trajectory = {make_pose(0)};
  slam::MapColorizer colorizer(make_config(), points, trajectory);

  auto raster = make_raster(100, 100, {0, 0, 0});
  for (std::uint32_t v = 0; v < 100; ++v) {
    for (std::uint32_t u = 51; u < 100; ++u) {
      set_pixel(raster, 100, u, v, {255, 255, 255});
    }
  }
  EXPECT_TRUE(colorizer.add_image(0, raster, 100, 100));

  const auto result = colorizer.finish();
  // The blend runs in linear light: halfway between black and white radiance
  // re-encodes to sRGB 188, not to the sRGB code midpoint 128.
  EXPECT_EQ(result.colors[0], (std::array<std::uint8_t, 3>{188, 188, 188}));
}

TEST(MapColorizer, InterpolatesCameraPoseBetweenTrajectoryPoints)
{
  // The trajectory translates +2 m in x over 1 s; at t = 0.5 s the camera sits
  // at x = +1, so the point at (1, 0, 5) is straight ahead (center pixel).
  const std::vector<std::array<float, 3>> points = {{1.0F, 0.0F, 5.0F}};
  const std::vector<TrajectoryPose> trajectory = {
    make_pose(0), make_pose(1'000'000'000, 2.0, 0.0, 0.0)};
  slam::MapColorizer colorizer(make_config(), points, trajectory);

  auto raster = make_raster(100, 100, {0, 0, 0});
  set_pixel(raster, 100, 50, 50, kRed);
  EXPECT_TRUE(colorizer.add_image(500'000'000, raster, 100, 100));

  const auto result = colorizer.finish();
  EXPECT_EQ(result.colors[0], kRed);
}

TEST(MapColorizer, AppliesTheCameraExtrinsic)
{
  // Camera mounted 1 m ahead of the cloud origin along +z: the point at
  // (0, 0, 5) sits 4 m in front of the camera, still at the center pixel.
  const std::vector<std::array<float, 3>> points = {{0.0F, 0.0F, 5.0F}};
  const std::vector<TrajectoryPose> trajectory = {make_pose(0)};
  auto config = make_config();
  config.t_cloud_cam.translation = {0.0, 0.0, 1.0};
  slam::MapColorizer colorizer(config, points, trajectory);

  EXPECT_TRUE(colorizer.add_image(0, make_raster(100, 100, kRed), 100, 100));

  const auto result = colorizer.finish();
  EXPECT_EQ(result.colors[0], kRed);
}

TEST(MapColorizer, ScalesIntrinsicsForAResizedImage)
{
  // CameraInfo is calibrated at 100x100 but the delivered image is 50x50: the
  // intrinsics are scaled so the center point still lands on the raster.
  const std::vector<std::array<float, 3>> points = {{0.0F, 0.0F, 5.0F}};
  const std::vector<TrajectoryPose> trajectory = {make_pose(0)};
  slam::MapColorizer colorizer(make_config(), points, trajectory);

  EXPECT_TRUE(colorizer.add_image(0, make_raster(50, 50, kRed), 50, 50));

  const auto result = colorizer.finish();
  EXPECT_EQ(result.colors[0], kRed);
}

TEST(MapColorizer, RejectsARasterSizeMismatch)
{
  const std::vector<std::array<float, 3>> points = {{0.0F, 0.0F, 5.0F}};
  const std::vector<TrajectoryPose> trajectory = {make_pose(0)};
  slam::MapColorizer colorizer(make_config(), points, trajectory);

  const auto raster = make_raster(100, 99, kRed);  // one row short
  EXPECT_FALSE(colorizer.add_image(0, raster, 100, 100));

  const auto result = colorizer.finish();
  EXPECT_EQ(result.images_used, 0U);
}

TEST(MapColorizer, FinishReportsObservedFlags)
{
  const std::vector<std::array<float, 3>> points = {{0.0F, 0.0F, 5.0F}, {0.0F, 0.0F, -5.0F}};
  const std::vector<TrajectoryPose> trajectory = {make_pose(0)};
  slam::MapColorizer colorizer(make_config(), points, trajectory);

  EXPECT_TRUE(colorizer.add_image(0, make_raster(100, 100, kRed), 100, 100));

  const auto result = colorizer.finish();
  ASSERT_EQ(result.observed.size(), 2U);
  EXPECT_EQ(result.observed[0], 1);  // in front of the camera: colored
  EXPECT_EQ(result.observed[1], 0);  // behind the camera: never observed
  ASSERT_EQ(result.weights.size(), 2U);
  EXPECT_FLOAT_EQ(result.weights[0], 1.0F);  // one unit-weight observation
  EXPECT_FLOAT_EQ(result.weights[1], 0.0F);
}

TEST(MapColorizer, WeightedNearObservationDominates)
{
  // The same point seen from 5 m and from 15 m with weights on and a short
  // distance reference: the near observation's weight dwarfs the far one's,
  // so the final color tracks the near view's red ramp, not the far blue.
  const std::vector<std::array<float, 3>> points = {{0.0F, 0.0F, 5.0F}};
  const std::vector<TrajectoryPose> trajectory = {
    make_pose(0), make_pose(1'000'000'000, 0.0, 0.0, -10.0)};
  auto config = make_config();
  config.use_weights = true;
  config.weight_distance_ref = 2.0;
  slam::MapColorizer colorizer(config, points, trajectory);

  // Horizontal ramps give the sharpness weight something to bite on; at the
  // center pixel both rasters sample value 100 in their dominant channel.
  auto red_ramp = make_raster(100, 100, {0, 0, 0});
  auto blue_ramp = make_raster(100, 100, {0, 0, 0});
  for (std::uint32_t v = 0; v < 100; ++v) {
    for (std::uint32_t u = 0; u < 100; ++u) {
      const auto level = static_cast<std::uint8_t>(2 * u);
      set_pixel(red_ramp, 100, u, v, {level, 0, 0});
      set_pixel(blue_ramp, 100, u, v, {0, 0, level});
    }
  }
  EXPECT_TRUE(colorizer.add_image(0, red_ramp, 100, 100));
  EXPECT_TRUE(colorizer.add_image(1'000'000'000, blue_ramp, 100, 100));

  const auto result = colorizer.finish();
  ASSERT_EQ(result.observed[0], 1);
  EXPECT_EQ(result.images_used, 2U);
  // Equal weights would land on (50, 0, 50); the near view must dominate.
  EXPECT_GT(result.colors[0][0], 70);
  EXPECT_LT(result.colors[0][2], 40);
  EXPECT_GT(result.colors[0][0], 3 * result.colors[0][2]);
}

TEST(MapColorizer, GainCompensationNeverDarkensABrighterFrame)
{
  // The same scene at double brightness in the second image: a below-1 gain
  // would drag the bright observations down toward the dark history and
  // ratchet the map toward black (a scene that genuinely brightens produces
  // exactly such ratios). The gain is clamped to >= 1, so the bright image
  // enters raw and the lit-mode anchor keeps its color.
  std::vector<std::array<float, 3>> points;
  for (int i = 0; i < 10; ++i) {
    points.push_back({static_cast<float>(-0.45 + 0.1 * i), 0.0F, 5.0F});
  }
  const std::vector<TrajectoryPose> trajectory = {make_pose(0), make_pose(1'000'000'000)};
  auto config = make_config();
  config.gain_compensation = true;
  config.gain_min_samples = 4;
  config.gain_min_prior_obs = 1;
  slam::MapColorizer colorizer(config, points, trajectory);

  EXPECT_TRUE(colorizer.add_image(0, make_raster(100, 100, {100, 20, 30}), 100, 100));
  EXPECT_TRUE(colorizer.add_image(1'000'000'000, make_raster(100, 100, {200, 40, 60}), 100, 100));

  const auto result = colorizer.finish();
  EXPECT_EQ(result.colored_points, 10U);
  for (std::size_t i = 0; i < 10; ++i) {
    EXPECT_EQ(result.colors[i], (std::array<std::uint8_t, 3>{200, 40, 60}))
      << "point " << i << " should keep the brighter frame's color";
  }
}

TEST(MapColorizer, GainCompensationTracksExposureChange)
{
  // Ten points in a row, observed by a bright image and then by the same
  // scene at half brightness. Gain compensation must scale the second
  // image's observations back up, so the final colors track the first image
  // instead of averaging down to (150, 30, 45).
  std::vector<std::array<float, 3>> points;
  for (int i = 0; i < 10; ++i) {
    points.push_back({static_cast<float>(-0.45 + 0.1 * i), 0.0F, 5.0F});
  }
  const std::vector<TrajectoryPose> trajectory = {make_pose(0), make_pose(1'000'000'000)};
  auto config = make_config();
  config.gain_compensation = true;
  config.gain_min_samples = 4;
  config.gain_min_prior_obs = 1;
  slam::MapColorizer colorizer(config, points, trajectory);

  EXPECT_TRUE(colorizer.add_image(0, make_raster(100, 100, {200, 40, 60}), 100, 100));
  EXPECT_TRUE(colorizer.add_image(1'000'000'000, make_raster(100, 100, {100, 20, 30}), 100, 100));

  const auto result = colorizer.finish();
  EXPECT_EQ(result.images_used, 2U);
  EXPECT_EQ(result.colored_points, 10U);
  for (std::size_t i = 0; i < 10; ++i) {
    EXPECT_EQ(result.colors[i], (std::array<std::uint8_t, 3>{200, 40, 60}))
      << "point " << i << " should track the first image's exposure";
  }
}

// Build a result by hand — merge_colorize_results consumes plain data, so the
// merge semantics are testable without running any projection.
slam::MapColorizeResult make_result(
  const std::vector<std::array<std::uint8_t, 3>> & colors,
  const std::vector<std::uint8_t> & observed, const std::vector<float> & weights,
  std::size_t images_used, std::size_t images_skipped)
{
  slam::MapColorizeResult result;
  result.colors = colors;
  result.observed = observed;
  result.weights = weights;
  for (const auto flag : observed) {
    result.colored_points += flag == 1 ? 1U : 0U;
  }
  result.images_used = images_used;
  result.images_skipped = images_skipped;
  return result;
}

TEST(MapColorizerMerge, BlendsObservedPointsByWeight)
{
  // Point 0: both cameras observed it with weights 3:1 -> weighted blend.
  // Point 1: only the second observed -> its color. Point 2: nobody -> gray.
  // A single shared point is too few samples for gain alignment, so no
  // scaling kicks in here.
  const auto first =
    make_result({{100, 100, 100}, kGray, kGray}, {1, 0, 0}, {3.0F, 0.0F, 0.0F}, 3, 1);
  const auto second =
    make_result({{200, 200, 200}, {30, 31, 32}, kGray}, {1, 1, 0}, {1.0F, 1.0F, 0.0F}, 4, 0);
  const std::array<slam::MapColorizeResult, 2> results{first, second};

  const auto merged = slam::merge_colorize_results(results);
  ASSERT_EQ(merged.colors.size(), 3U);
  // The blend runs in linear light: (3 * linear(100) + 1 * linear(200)) / 4
  // re-encodes to 134 per channel (an sRGB-space blend would give 125).
  EXPECT_EQ(merged.colors[0], (std::array<std::uint8_t, 3>{134, 134, 134}));
  EXPECT_EQ(merged.colors[1], (std::array<std::uint8_t, 3>{30, 31, 32}));
  EXPECT_EQ(merged.colors[2], kGray);
  EXPECT_EQ(merged.observed, (std::vector<std::uint8_t>{1, 1, 0}));
  EXPECT_EQ(merged.weights, (std::vector<float>{4.0F, 1.0F, 0.0F}));
  EXPECT_EQ(merged.colored_points, 2U);
  EXPECT_EQ(merged.images_used, 7U);
  EXPECT_EQ(merged.images_skipped, 1U);
}

TEST(MapColorizerMerge, AlignsCameraGainBeforeBlending)
{
  // 100 points seen by both cameras plus one seen only by the second. Camera
  // 2 is uniformly half as bright as camera 1; with enough shared samples
  // the merge scales camera 2 up to camera 1's exposure before blending, so
  // every blended color lands on camera 1's (200, 100, 52).
  constexpr std::size_t kShared = 100;
  auto colors_first = std::vector<std::array<std::uint8_t, 3>>(kShared + 1, {200, 100, 52});
  auto observed_first = std::vector<std::uint8_t>(kShared + 1, 1);
  auto weights_first = std::vector<float>(kShared + 1, 1.0F);
  colors_first[kShared] = kGray;
  observed_first[kShared] = 0;
  weights_first[kShared] = 0.0F;
  const auto first = make_result(colors_first, observed_first, weights_first, 3, 1);
  const auto second = make_result(
    std::vector<std::array<std::uint8_t, 3>>(kShared + 1, {100, 50, 26}),
    std::vector<std::uint8_t>(kShared + 1, 1), std::vector<float>(kShared + 1, 1.0F), 4, 0);
  const std::array<slam::MapColorizeResult, 2> results{first, second};

  const auto merged = slam::merge_colorize_results(results);
  ASSERT_EQ(merged.colors.size(), kShared + 1);
  EXPECT_EQ(merged.colors[0], (std::array<std::uint8_t, 3>{200, 100, 52}));
  EXPECT_EQ(merged.colors[kShared], (std::array<std::uint8_t, 3>{200, 100, 52}));
  EXPECT_FLOAT_EQ(merged.weights[0], 2.0F);
  EXPECT_FLOAT_EQ(merged.weights[kShared], 1.0F);
  EXPECT_EQ(merged.observed, std::vector<std::uint8_t>(kShared + 1, 1));
  EXPECT_EQ(merged.colored_points, kShared + 1);
  EXPECT_EQ(merged.images_used, 7U);
  EXPECT_EQ(merged.images_skipped, 1U);
}

TEST(MapColorizerMerge, EmptyInputYieldsEmptyResult)
{
  const auto merged = slam::merge_colorize_results({});
  EXPECT_TRUE(merged.colors.empty());
  EXPECT_TRUE(merged.observed.empty());
  EXPECT_TRUE(merged.weights.empty());
  EXPECT_EQ(merged.colored_points, 0U);
}

TEST(MapColorizer, MultithreadedRunMatchesSingleThread)
{
  // A deterministic spread of points in front of the camera, colored from a
  // gradient raster with splatting on (the default path); the per-point
  // result must not depend on the thread count.
  std::vector<std::array<float, 3>> points;
  for (int i = 0; i < 500; ++i) {
    const float x = static_cast<float>((i % 25) - 12) * 0.1F;
    const float y = static_cast<float>((i / 25) - 10) * 0.1F;
    const float z = 4.0F + static_cast<float>(i % 7) * 0.5F;
    points.push_back({x, y, z});
  }
  const std::vector<TrajectoryPose> trajectory = {make_pose(0), make_pose(1'000'000'000)};

  auto gradient = make_raster(100, 100, {0, 0, 0});
  for (std::uint32_t v = 0; v < 100; ++v) {
    for (std::uint32_t u = 0; u < 100; ++u) {
      set_pixel(
        gradient, 100, u, v,
        {static_cast<std::uint8_t>(2 * u), static_cast<std::uint8_t>(2 * v), 77});
    }
  }

  auto run = [&](int threads) {
    auto config = make_config();
    config.rasterizer.splat = true;
    config.rasterizer.num_threads = threads;
    slam::MapColorizer colorizer(config, points, trajectory);
    colorizer.add_image(0, gradient, 100, 100);
    colorizer.add_image(1'000'000'000, gradient, 100, 100);
    return colorizer.finish();
  };

  const auto serial = run(1);
  const auto parallel = run(4);
  EXPECT_GT(serial.colored_points, 0U);
  EXPECT_EQ(serial.colors, parallel.colors);
  EXPECT_EQ(serial.observed, parallel.observed);
  EXPECT_EQ(serial.weights, parallel.weights);
  EXPECT_EQ(serial.colored_points, parallel.colored_points);
}

}  // namespace
