// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/cloud_mapper.hpp"

#include "bagwiz/core/image/camera_info.hpp"
#include "bagwiz/core/image/packed_raster.hpp"
#include "bagwiz/core/slam/lidar_scan.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

// Integration test that drives the real GLIM SubMapping -> GlobalMapping
// pipeline through CloudMapper. Compiled only when BAGWIZ_WITH_SLAM is on (it
// links the vendored GLIM stack). A static sensor observing a fixed structured
// scene must yield a non-empty, finite optimized map plus a non-empty,
// time-monotonic, bounded trajectory — no claim about absolute accuracy, just
// that the in-process mapping pipeline runs and produces a sane result.
namespace
{
namespace slam = bagwiz::core::slam;

// A 10 x 10 x ~3 m "room": a floor grid plus four walls. Dense enough geometric
// structure for scan-to-model matching, expressed in the (static) sensor frame.
// Mirrors cloud_odometry_test's scene.
slam::LidarScan make_room_scan(std::int64_t stamp_ns)
{
  slam::LidarScan scan;
  scan.stamp_ns = stamp_ns;
  scan.frame_id = "lidar";

  constexpr double kHalf = 5.0;
  constexpr double kHeight = 3.0;
  constexpr int kN = 20;
  const auto lerp = [](double a, double b, int i, int n) {
    return a + (b - a) * static_cast<double>(i) / static_cast<double>(n - 1);
  };

  for (int i = 0; i < kN; ++i) {
    for (int j = 0; j < kN; ++j) {
      scan.points.push_back({lerp(-kHalf, kHalf, i, kN), lerp(-kHalf, kHalf, j, kN), -1.0});
    }
  }
  for (int i = 0; i < kN; ++i) {
    for (int k = 0; k < kN; ++k) {
      const double u = lerp(-kHalf, kHalf, i, kN);
      const double z = lerp(-1.0, kHeight - 1.0, k, kN);
      scan.points.push_back({u, -kHalf, z});
      scan.points.push_back({u, kHalf, z});
      scan.points.push_back({-kHalf, u, z});
      scan.points.push_back({kHalf, u, z});
    }
  }
  return scan;
}

TEST(CloudMapper, StationarySensorYieldsMapAndTrajectory)
{
  // Submaps form only after enough keyframes accumulate, and CT odometry
  // finalizes frames only as they leave its fixed-lag window. Feed a long
  // sequence (120 scans @ 10 Hz = 12 s) so at least one submap is created and
  // global optimization has something to optimize.
  slam::CloudMapper mapper;
  constexpr std::int64_t kDtNs = 100'000'000;  // 10 Hz
  std::int64_t stamp = 1'000'000'000'000'000'000LL;
  for (int i = 0; i < 120; ++i) {
    mapper.insert(make_room_scan(stamp));
    stamp += kDtNs;
  }

  const slam::CloudMap map = mapper.finish();

  // The optimized map is non-empty and every point is finite.
  ASSERT_FALSE(map.points.empty());
  for (const auto & p : map.points) {
    ASSERT_TRUE(std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]));
  }
  // Intensities are all-or-nothing: either empty or one per point.
  EXPECT_TRUE(map.intensities.empty() || map.intensities.size() == map.points.size());

  // The map of a 10 x 10 x ~3 m room should stay within a generous bounding box
  // even after global optimization (loose: this is a sanity bound, not an
  // accuracy claim).
  for (const auto & p : map.points) {
    EXPECT_LT(std::abs(p[0]), 20.0f);
    EXPECT_LT(std::abs(p[1]), 20.0f);
    EXPECT_LT(std::abs(p[2]), 20.0f);
  }

  // The globally-optimized trajectory is non-empty, time-monotonic, finite, and
  // bounded for a stationary sensor.
  ASSERT_FALSE(map.trajectory.empty());
  for (std::size_t i = 1; i < map.trajectory.size(); ++i) {
    EXPECT_LT(map.trajectory[i - 1].timestamp_ns, map.trajectory[i].timestamp_ns);
  }

  double min_x = map.trajectory.front().tx;
  double max_x = min_x;
  double min_y = map.trajectory.front().ty;
  double max_y = min_y;
  double min_z = map.trajectory.front().tz;
  double max_z = min_z;
  for (const auto & pose : map.trajectory) {
    ASSERT_TRUE(std::isfinite(pose.tx) && std::isfinite(pose.ty) && std::isfinite(pose.tz));
    ASSERT_TRUE(std::isfinite(pose.qx) && std::isfinite(pose.qy));
    ASSERT_TRUE(std::isfinite(pose.qz) && std::isfinite(pose.qw));
    min_x = std::min(min_x, pose.tx);
    max_x = std::max(max_x, pose.tx);
    min_y = std::min(min_y, pose.ty);
    max_y = std::max(max_y, pose.ty);
    min_z = std::min(min_z, pose.tz);
    max_z = std::max(max_z, pose.tz);
  }
  EXPECT_LT(max_x - min_x, 2.0) << "stationary trajectory drifted in x";
  EXPECT_LT(max_y - min_y, 2.0) << "stationary trajectory drifted in y";
  EXPECT_LT(max_z - min_z, 2.0) << "stationary trajectory drifted in z";
}

namespace image = bagwiz::core::image;

image::CameraInfo make_test_camera_info(std::uint32_t width, std::uint32_t height)
{
  image::CameraInfo info;
  info.width = width;
  info.height = height;
  info.k = {10.0, 0.0, width / 2.0, 0.0, 10.0, height / 2.0, 0.0, 0.0, 1.0};
  info.frame_id = "lidar";
  return info;
}

image::PackedRaster make_solid_image(
  std::uint32_t width, std::uint32_t height, const std::array<std::uint8_t, 3> & bgr)
{
  image::PackedRaster raster;
  raster.width = width;
  raster.height = height;
  raster.encoding = "bgr8";
  raster.bgr.assign(static_cast<std::size_t>(width) * height * 3U, std::byte{bgr[0]});
  for (std::size_t i = 1; i < 3; ++i) {
    for (std::size_t p = 0; p < static_cast<std::size_t>(width) * height; ++p) {
      raster.bgr[p * 3U + i] = std::byte{bgr[i]};
    }
  }
  return raster;
}

TEST(CloudMapper, CameraColorsTheExportedMap)
{
  // Camera is co-located with the LiDAR (identity rotation) but shifted 10 m
  // behind it along z, so the static room scene is 9–12 m in front of the
  // camera and projects inside the image. Every pixel is red; the exported map
  // should therefore carry a red BGR color for every voxel.
  slam::CloudMapperConfig config;
  config.camera = slam::CloudMapperConfig::CameraConfig{};
  config.camera->info = make_test_camera_info(64, 48);
  config.camera->t_lidar_camera = {
    1.0, 0.0, 0.0,  0.0,  // column 0
    0.0, 1.0, 0.0,  0.0,  // column 1
    0.0, 0.0, 1.0,  0.0,  // column 2
    0.0, 0.0, 10.0, 1.0   // translation: camera 10 m behind lidar along z
  };
  config.camera->use_rectified = false;

  slam::CloudMapper mapper(config);
  constexpr std::int64_t kDtNs = 100'000'000;  // 10 Hz
  std::int64_t stamp = 1'000'000'000'000'000'000LL;
  constexpr std::array<std::uint8_t, 3> kRed{0, 0, 255};
  for (int i = 0; i < 120; ++i) {
    mapper.insert_image(stamp, make_solid_image(64, 48, kRed));
    mapper.insert(make_room_scan(stamp));
    stamp += kDtNs;
  }

  const slam::CloudMap map = mapper.finish();

  ASSERT_FALSE(map.points.empty());
  ASSERT_EQ(map.colors.size(), map.points.size());
  for (const auto & c : map.colors) {
    EXPECT_EQ(c, kRed);
  }
}

}  // namespace
