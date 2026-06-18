// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/cloud_odometry.hpp"

#include "bagwiz/core/slam/lidar_scan.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

// Integration test that drives the real GLIM OdometryEstimationCT through
// CloudOdometry. Compiled only when BAGWIZ_WITH_SLAM is on (it links the
// vendored GLIM stack). A static sensor observing a fixed structured scene must
// yield a non-empty, time-monotonic trajectory that stays near the origin — no
// claim about absolute accuracy, just that the in-process pipeline runs and
// produces a sane result.
namespace
{
namespace slam = bagwiz::core::slam;

// A 10 x 10 x ~3 m "room": a floor grid plus four walls. Dense enough geometric
// structure for scan-to-model matching, expressed in the (static) sensor frame.
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

TEST(CloudOdometry, StationarySensorYieldsStableTrajectory)
{
  // GLIM's CT odometry finalizes a frame only once it leaves the fixed-lag
  // window (default smoother_lag = 5.0 s, and CT does not flush the window in
  // get_remaining_frames). Feed well past 5 s (80 scans @ 10 Hz = 8 s) so frames
  // marginalize and a trajectory is produced.
  slam::CloudOdometry odometry;
  constexpr std::int64_t kDtNs = 100'000'000;  // 10 Hz
  std::int64_t stamp = 1'000'000'000'000'000'000LL;
  for (int i = 0; i < 80; ++i) {
    odometry.insert(make_room_scan(stamp));
    stamp += kDtNs;
  }

  const auto trajectory = odometry.finish();
  ASSERT_FALSE(trajectory.empty());

  for (std::size_t i = 1; i < trajectory.size(); ++i) {
    EXPECT_LT(trajectory[i - 1].timestamp_ns, trajectory[i].timestamp_ns);
  }

  // A stationary sensor must yield a BOUNDED, finite trajectory — the in-process
  // GLIM pipeline runs without diverging or producing NaNs. We bound the spatial
  // EXTENT (not distance from the world origin — GLIM anchors its own origin a
  // fixed offset away). The bound is loose: a symmetric, noise-free synthetic
  // scene with no IMU is weakly constrained, so CT wanders ~1 m; tight accuracy
  // is validated end-to-end on real data, not asserted here.
  double min_x = trajectory.front().tx;
  double max_x = min_x;
  double min_y = trajectory.front().ty;
  double max_y = min_y;
  double min_z = trajectory.front().tz;
  double max_z = min_z;
  for (const auto & pose : trajectory) {
    min_x = std::min(min_x, pose.tx);
    max_x = std::max(max_x, pose.tx);
    min_y = std::min(min_y, pose.ty);
    max_y = std::max(max_y, pose.ty);
    min_z = std::min(min_z, pose.tz);
    max_z = std::max(max_z, pose.tz);
    ASSERT_TRUE(std::isfinite(pose.tx) && std::isfinite(pose.ty) && std::isfinite(pose.tz));
  }
  EXPECT_LT(max_x - min_x, 2.0) << "stationary trajectory drifted in x";
  EXPECT_LT(max_y - min_y, 2.0) << "stationary trajectory drifted in y";
  EXPECT_LT(max_z - min_z, 2.0) << "stationary trajectory drifted in z";
}

}  // namespace
