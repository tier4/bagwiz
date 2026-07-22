// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "map_slam_colorize.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

#include "map_slam_threads.hpp"  // NOLINT(build/include_subdir) src-local shared header under test

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <thread>
#include <vector>

namespace
{
using bagwiz::commands::build_camera_colorizers;
using bagwiz::commands::build_shared_colorize_geometry;
using bagwiz::commands::colorize_thread_count;
using bagwiz::commands::resolve_threads;
using bagwiz::core::TrajectoryPose;

TrajectoryPose make_pose(std::int64_t stamp_ns)
{
  TrajectoryPose pose;
  pose.timestamp_ns = stamp_ns;
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

TEST(ColorizeThreadCount, PositivePassesThroughTheHardwareCap)
{
  EXPECT_EQ(colorize_thread_count(1), 1);
  const unsigned int hardware = std::thread::hardware_concurrency();
  if (hardware > 0) {
    EXPECT_EQ(colorize_thread_count(std::numeric_limits<int>::max()), static_cast<int>(hardware));
  }
}

TEST(ResolveThreads, ZeroResolvesToTheHardwareConcurrency)
{
  const unsigned int hardware = std::thread::hardware_concurrency();
  const int expected = hardware > 0 ? static_cast<int>(hardware) : 1;
  EXPECT_EQ(resolve_threads(0), expected);
  EXPECT_EQ(resolve_threads(-3), expected);
}

TEST(ResolveThreads, PositiveValuesPassThroughUnderTheCap)
{
  EXPECT_EQ(resolve_threads(1), 1);
  const unsigned int hardware = std::thread::hardware_concurrency();
  if (hardware > 1) {
    EXPECT_EQ(resolve_threads(static_cast<int>(hardware) + 1), static_cast<int>(hardware));
  }
}

TEST(BuildSharedColorizeGeometry, CoversEveryPoint)
{
  const std::vector<std::array<float, 3>> points = {
    {0.0F, 0.0F, 5.0F}, {0.1F, 0.0F, 5.0F}, {0.0F, 0.1F, 5.0F}, {0.1F, 0.1F, 5.0F}};
  const auto geometry = build_shared_colorize_geometry(points, 1);
  EXPECT_EQ(geometry->normals.size(), points.size());
  EXPECT_EQ(geometry->spacings.size(), points.size());
}

TEST(BuildCameraColorizers, BuildsOneWorkingColorizerPerCamera)
{
  const std::vector<bagwiz::core::image::CameraInfo> camera_infos = {
    make_pinhole(), make_pinhole()};
  const std::vector<bagwiz::core::slam::SensorTransform> t_cloud_cams = {{}, {}};
  const std::vector<std::array<float, 3>> points = {{0.0F, 0.0F, 5.0F}};
  const std::vector<TrajectoryPose> trajectory = {make_pose(0), make_pose(1'000'000'000)};
  const auto geometry = build_shared_colorize_geometry(points, 1);

  auto colorizers = build_camera_colorizers(
    camera_infos, t_cloud_cams, 100.0, 1, false, geometry, points, trajectory);
  ASSERT_EQ(colorizers.size(), 2U);

  // Wiring smoke: an in-span, correctly-sized image is accepted and reduces.
  const std::vector<std::byte> raster(
    static_cast<std::size_t>(100) * 3U * 100U, static_cast<std::byte>(0x7F));
  EXPECT_TRUE(colorizers[0]->add_image(0, raster, 100, 100));
  const auto result = colorizers[0]->finish();
  EXPECT_EQ(result.colors.size(), points.size());
  EXPECT_EQ(result.images_used, 1U);
  EXPECT_EQ(result.images_skipped, 0U);
}

}  // namespace
