// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/cloud_mapper.hpp"

#include "bagwiz/core/slam/lidar_scan.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

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

// Same room, with a constant intensity attached to every point. A constant
// survives voxel-grid averaging exactly, so the exported map's intensities must
// all equal it — a tight check that intensity flows end to end.
slam::LidarScan make_room_scan_with_intensity(std::int64_t stamp_ns, float intensity)
{
  slam::LidarScan scan = make_room_scan(stamp_ns);
  scan.intensities.assign(scan.points.size(), static_cast<double>(intensity));
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

// Regression: in LiDAR-only mode the odometry backend is GLIM's
// OdometryEstimationCT, which (unlike the LiDAR-IMU backend) never copies
// per-point intensities onto its estimation-frame cloud. The mapper must still
// export intensities by sourcing them from the preprocessed frame, so a scan fed
// with intensities yields a map that carries them — not an empty intensity set.
TEST(CloudMapper, LidarOnlyPreservesIntensity)
{
  constexpr float kIntensity = 7.0f;
  slam::CloudMapper mapper;                    // no extrinsic => LiDAR-only (CT) backend
  constexpr std::int64_t kDtNs = 100'000'000;  // 10 Hz
  std::int64_t stamp = 1'000'000'000'000'000'000LL;
  for (int i = 0; i < 120; ++i) {
    mapper.insert(make_room_scan_with_intensity(stamp, kIntensity));
    stamp += kDtNs;
  }

  const slam::CloudMap map = mapper.finish();

  ASSERT_FALSE(map.points.empty());
  // The defect: intensities came back empty in LiDAR-only mode. They must be
  // present and one-per-point.
  ASSERT_EQ(map.intensities.size(), map.points.size());
  // A constant intensity is invariant under per-voxel averaging, so every
  // exported value must equal what was fed (and crucially must not be zero).
  for (const float v : map.intensities) {
    EXPECT_NEAR(v, kIntensity, 1e-3f);
  }
}

}  // namespace
