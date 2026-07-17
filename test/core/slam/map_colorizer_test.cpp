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

slam::MapColorizerConfig make_config()
{
  slam::MapColorizerConfig config;
  config.camera = make_pinhole();
  config.num_threads = 1;
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

TEST(MapColorizer, OcclusionBlocksFarPointOnSameRay)
{
  const std::vector<std::array<float, 3>> points = {{0.0F, 0.0F, 5.0F}, {0.0F, 0.0F, 10.0F}};
  const std::vector<TrajectoryPose> trajectory = {make_pose(0)};
  slam::MapColorizer colorizer(make_config(), points, trajectory);

  const auto raster = make_raster(100, 100, kRed);
  EXPECT_TRUE(colorizer.add_image(0, raster, 100, 100));

  const auto result = colorizer.finish();
  EXPECT_EQ(result.colors[0], kRed);
  EXPECT_EQ(result.colors[1], kGray);
  EXPECT_EQ(result.colored_points, 1U);
}

TEST(MapColorizer, AveragesObservationsAcrossImages)
{
  const std::vector<std::array<float, 3>> points = {{0.0F, 0.0F, 5.0F}};
  const std::vector<TrajectoryPose> trajectory = {make_pose(0), make_pose(1'000'000'000)};
  slam::MapColorizer colorizer(make_config(), points, trajectory);

  EXPECT_TRUE(colorizer.add_image(0, make_raster(100, 100, {255, 0, 0}), 100, 100));
  EXPECT_TRUE(colorizer.add_image(1'000'000'000, make_raster(100, 100, {0, 0, 255}), 100, 100));

  const auto result = colorizer.finish();
  // Rounded average of (255, 0, 0) and (0, 0, 255).
  EXPECT_EQ(result.colors[0], (std::array<std::uint8_t, 3>{128, 0, 128}));
  EXPECT_EQ(result.images_used, 2U);
}

TEST(MapColorizer, CullsPointsBeyondMaxRange)
{
  const std::vector<std::array<float, 3>> points = {{0.0F, 0.0F, 5.0F}};
  const std::vector<TrajectoryPose> trajectory = {make_pose(0)};
  auto config = make_config();
  config.max_range = 3.0;
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

TEST(MapColorizer, MultithreadedRunMatchesSingleThread)
{
  // A deterministic spread of points in front of the camera, colored from a
  // gradient raster; the per-point average must not depend on the thread count.
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
    config.num_threads = threads;
    slam::MapColorizer colorizer(config, points, trajectory);
    colorizer.add_image(0, gradient, 100, 100);
    colorizer.add_image(1'000'000'000, gradient, 100, 100);
    return colorizer.finish();
  };

  const auto serial = run(1);
  const auto parallel = run(4);
  EXPECT_GT(serial.colored_points, 0U);
  EXPECT_EQ(serial.colors, parallel.colors);
  EXPECT_EQ(serial.colored_points, parallel.colored_points);
}

}  // namespace
