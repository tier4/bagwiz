// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/cloud_filters.hpp"
#include "bagwiz/core/slam/cloud_mapper.hpp"
#include "bagwiz/core/slam/cloud_voxelize_gpu.hpp"
#include "bagwiz/core/slam/lidar_scan.hpp"

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

// GPU-path integration + unit tests. Compiled only under BAGWIZ_WITH_SLAM_CUDA
// (the CMake target gates on it), and every test self-skips when no CUDA device
// is visible — so a CUDA build on a GPU-less runner (and CI, which never sets the
// flag) still passes. Mirrors cloud_mapper_test.cpp's room-scene fixture.
namespace
{
namespace slam = bagwiz::core::slam;

bool cuda_available()
{
  int count = 0;
  const bool ok = cudaGetDeviceCount(&count) == cudaSuccess && count > 0;
  cudaGetLastError();  // clear any sticky "no device" error.
  return ok;
}

// Same 10 x 10 x ~3 m "room" used by the CPU tests: floor grid + four walls.
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

// Sort a point set lexicographically so two voxelizations (GPU sorted-key order
// vs CPU first-seen order) can be compared element-wise.
std::vector<std::array<float, 3>> sorted(std::vector<std::array<float, 3>> v)
{
  std::sort(v.begin(), v.end(), [](const auto & a, const auto & b) {
    if (a[0] != b[0]) {
      return a[0] < b[0];
    }
    if (a[1] != b[1]) {
      return a[1] < b[1];
    }
    return a[2] < b[2];
  });
  return v;
}

// The GPU voxelizer must reduce the same points into the same voxels as the CPU
// VoxelGrid: identical occupied-voxel count and matching centroids (within float
// precision; CPU accumulates in double, GPU in float). Points are placed near
// voxel centers so the two floor() binnings agree exactly (no boundary ambiguity).
TEST(CloudVoxelizeGpu, MatchesCpuVoxelGrid)
{
  if (!cuda_available()) {
    GTEST_SKIP() << "no CUDA device";
  }
  constexpr double kRes = 0.5;
  std::vector<std::array<float, 3>> pts;
  std::vector<float> ints;
  // 8 x 8 x 4 voxels, ~12 points each, jittered around the voxel center.
  for (int ix = 0; ix < 8; ++ix) {
    for (int iy = 0; iy < 8; ++iy) {
      for (int iz = 0; iz < 4; ++iz) {
        const float cx = (static_cast<float>(ix) + 0.5F) * static_cast<float>(kRes);
        const float cy = (static_cast<float>(iy) + 0.5F) * static_cast<float>(kRes);
        const float cz = (static_cast<float>(iz) + 0.5F) * static_cast<float>(kRes);
        for (int s = 0; s < 12; ++s) {
          const float j = (static_cast<float>(s) - 5.5F) * 0.01F;  // |j| <= 0.055 < res/2
          pts.push_back({cx + j, cy - j, cz + j});
          ints.push_back(static_cast<float>(ix + iy + iz));
        }
      }
    }
  }

  slam::VoxelGrid grid(kRes, true);
  for (std::size_t i = 0; i < pts.size(); ++i) {
    grid.add(pts[i][0], pts[i][1], pts[i][2], ints[i]);
  }
  const auto cpu_pts = sorted(grid.points());

  std::vector<std::array<float, 3>> gpu_pts;
  std::vector<float> gpu_ints;
  ASSERT_TRUE(slam::voxelize_gpu(pts, ints, kRes, gpu_pts, gpu_ints));
  ASSERT_EQ(gpu_pts.size(), gpu_ints.size());
  ASSERT_EQ(gpu_pts.size(), cpu_pts.size()) << "GPU and CPU voxel counts differ";

  const auto gpu_sorted = sorted(gpu_pts);
  for (std::size_t i = 0; i < cpu_pts.size(); ++i) {
    EXPECT_NEAR(gpu_sorted[i][0], cpu_pts[i][0], 1e-3F);
    EXPECT_NEAR(gpu_sorted[i][1], cpu_pts[i][1], 1e-3F);
    EXPECT_NEAR(gpu_sorted[i][2], cpu_pts[i][2], 1e-3F);
  }
}

// An empty input is a successful no-op (not a failure).
TEST(CloudVoxelizeGpu, EmptyInputSucceeds)
{
  if (!cuda_available()) {
    GTEST_SKIP() << "no CUDA device";
  }
  std::vector<std::array<float, 3>> out_pts{{1, 2, 3}};
  std::vector<float> out_ints{4};
  ASSERT_TRUE(slam::voxelize_gpu({}, {}, 0.2, out_pts, out_ints));
  EXPECT_TRUE(out_pts.empty());
  EXPECT_TRUE(out_ints.empty());
}

// End-to-end GPU mapping: a stationary sensor on the room scene with use_gpu=true
// runs CT odometry + VGICP_GPU sub/global mapping + GPU export voxelization and
// must yield a non-empty, finite optimized map and a non-empty, time-monotonic
// trajectory (no accuracy claim; just that the GPU pipeline runs end to end).
TEST(CloudMapperGpu, StationarySensorYieldsMapAndTrajectory)
{
  if (!cuda_available()) {
    GTEST_SKIP() << "no CUDA device";
  }
  slam::CloudMapperConfig config;
  config.use_gpu = true;  // LiDAR-only -> CT odometry, GPU registration + voxelization
  slam::CloudMapper mapper(config);

  constexpr std::int64_t kDtNs = 100'000'000;  // 10 Hz
  std::int64_t stamp = 1'000'000'000'000'000'000LL;
  for (int i = 0; i < 120; ++i) {
    mapper.insert(make_room_scan(stamp));
    stamp += kDtNs;
  }

  const slam::CloudMap map = mapper.finish();

  ASSERT_FALSE(map.points.empty());
  for (const auto & p : map.points) {
    ASSERT_TRUE(std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]));
    EXPECT_LT(std::abs(p[0]), 20.0F);
    EXPECT_LT(std::abs(p[1]), 20.0F);
    EXPECT_LT(std::abs(p[2]), 20.0F);
  }
  EXPECT_TRUE(map.intensities.empty() || map.intensities.size() == map.points.size());

  ASSERT_FALSE(map.trajectory.empty());
  for (std::size_t i = 1; i < map.trajectory.size(); ++i) {
    EXPECT_LT(map.trajectory[i - 1].timestamp_ns, map.trajectory[i].timestamp_ns);
  }
}

}  // namespace
