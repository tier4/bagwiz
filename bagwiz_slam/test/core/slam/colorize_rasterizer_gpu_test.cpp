// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/colorize_rasterizer_gpu.hpp"

#include "bagwiz/core/slam/colorize_rasterizer.hpp"

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <utility>
#include <vector>

namespace
{

namespace slam = bagwiz::core::slam;

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

slam::ColorizeView make_center_view()
{
  slam::ColorizeView view;
  view.r_cam_world = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  view.t_cam_world = {0.0, 0.0, 0.0};
  view.camera = make_pinhole();
  view.width = 100;
  view.height = 100;
  return view;
}

std::vector<std::uint32_t> sorted_indices(const std::vector<slam::VisiblePoint> & points)
{
  std::vector<std::uint32_t> indices;
  indices.reserve(points.size());
  for (const auto & p : points) {
    indices.push_back(p.index);
  }
  std::sort(indices.begin(), indices.end());
  return indices;
}

class ColorizeRasterizerGpuTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    int device_count = 0;
    if (cudaGetDeviceCount(&device_count) != cudaSuccess || device_count <= 0) {
      GTEST_SKIP() << "No CUDA device available";
    }
  }
};

TEST_F(ColorizeRasterizerGpuTest, SingleCenterPointMatchesCpu)
{
  const std::vector<std::array<float, 3>> points = {{0.0F, 0.0F, 5.0F}};
  const std::vector<float> spacings = {0.1F};
  slam::ColorizeRasterizerConfig config;
  config.splat = false;

  auto cpu = slam::make_cpu_colorize_rasterizer(points, spacings, config);
  auto gpu = slam::make_gpu_colorize_rasterizer(points, spacings, config);
  ASSERT_NE(gpu, nullptr);

  std::vector<slam::VisiblePoint> cpu_visible;
  std::vector<slam::VisiblePoint> gpu_visible;
  cpu->visible_points(make_center_view(), {}, cpu_visible);
  gpu->visible_points(make_center_view(), {}, gpu_visible);

  ASSERT_EQ(cpu_visible.size(), 1U);
  ASSERT_EQ(gpu_visible.size(), 1U);
  EXPECT_EQ(gpu_visible[0].index, 0U);
  EXPECT_NEAR(gpu_visible[0].u, 50.0, 1e-6);
  EXPECT_NEAR(gpu_visible[0].v, 50.0, 1e-6);
  EXPECT_NEAR(gpu_visible[0].depth, 5.0F, 1e-4F);
}

TEST_F(ColorizeRasterizerGpuTest, OcclusionMatchesCpu)
{
  // A dense 5x5 wall of points at z = 5 covers the pixels around the image
  // center; the far point at z = 10 projects to the same pixel and is rejected.
  std::vector<std::array<float, 3>> points;
  for (const float gx : {-0.1F, -0.05F, 0.0F, 0.05F, 0.1F}) {
    for (const float gy : {-0.1F, -0.05F, 0.0F, 0.05F, 0.1F}) {
      points.push_back({gx, gy, 5.0F});
    }
  }
  points.push_back({0.0F, 0.0F, 10.0F});
  std::vector<float> spacings(points.size(), 0.05F);
  slam::ColorizeRasterizerConfig config;
  config.splat = false;

  auto cpu = slam::make_cpu_colorize_rasterizer(points, spacings, config);
  auto gpu = slam::make_gpu_colorize_rasterizer(points, spacings, config);
  ASSERT_NE(gpu, nullptr);

  std::vector<slam::VisiblePoint> cpu_visible;
  std::vector<slam::VisiblePoint> gpu_visible;
  cpu->visible_points(make_center_view(), {}, cpu_visible);
  gpu->visible_points(make_center_view(), {}, gpu_visible);

  const auto cpu_idx = sorted_indices(cpu_visible);
  const auto gpu_idx = sorted_indices(gpu_visible);
  EXPECT_EQ(cpu_idx, gpu_idx);
  EXPECT_FALSE(std::binary_search(gpu_idx.begin(), gpu_idx.end(), 25U));  // far point occluded
}

TEST_F(ColorizeRasterizerGpuTest, SplatClosesHolesBetweenSparseOccluders)
{
  const std::vector<std::array<float, 3>> points = {
    {-0.05F, -0.05F, 5.0F},
    {0.05F, -0.05F, 5.0F},
    {-0.05F, 0.05F, 5.0F},
    {0.05F, 0.05F, 5.0F},
    {0.0F, 0.0F, 10.0F}};
  // Spacing large enough that the four front points' splat discs cover the
  // far point at pixel (50, 50). With f_avg = 100, depth = 5, spacing = 0.15
  // gives a 1.5 px radius; the far point is sqrt(2) ~ 1.41 px from each corner.
  std::vector<float> spacings(points.size(), 0.15F);

  auto run = [&](bool splat) {
    slam::ColorizeRasterizerConfig config;
    config.splat = splat;
    auto cpu = slam::make_cpu_colorize_rasterizer(points, spacings, config);
    auto gpu = slam::make_gpu_colorize_rasterizer(points, spacings, config);
    EXPECT_NE(gpu, nullptr);
    std::vector<slam::VisiblePoint> cpu_visible;
    std::vector<slam::VisiblePoint> gpu_visible;
    cpu->visible_points(make_center_view(), {}, cpu_visible);
    gpu->visible_points(make_center_view(), {}, gpu_visible);
    return std::make_pair(sorted_indices(cpu_visible), sorted_indices(gpu_visible));
  };

  const auto with_splat = run(true);
  EXPECT_EQ(with_splat.first, with_splat.second);
  EXPECT_FALSE(std::binary_search(with_splat.second.begin(), with_splat.second.end(), 4U));

  const auto without_splat = run(false);
  EXPECT_EQ(without_splat.first, without_splat.second);
  EXPECT_TRUE(std::binary_search(without_splat.second.begin(), without_splat.second.end(), 4U));
}

TEST_F(ColorizeRasterizerGpuTest, DynamicOccluderRejectsFarPoint)
{
  const std::vector<std::array<float, 3>> points = {{0.0F, 0.0F, 10.0F}};
  const std::vector<std::array<float, 3>> dynamic = {
    {0.0F, 0.0F, 5.0F},  // occluder on the same ray
    {0.0F, 0.0F, -5.0F}  // behind camera: ignored
  };
  std::vector<float> spacings = {0.05F};
  slam::ColorizeRasterizerConfig config;
  config.splat = false;

  auto cpu = slam::make_cpu_colorize_rasterizer(points, spacings, config);
  auto gpu = slam::make_gpu_colorize_rasterizer(points, spacings, config);
  ASSERT_NE(gpu, nullptr);

  std::vector<slam::VisiblePoint> cpu_visible;
  std::vector<slam::VisiblePoint> gpu_visible;
  cpu->visible_points(make_center_view(), dynamic, cpu_visible);
  gpu->visible_points(make_center_view(), dynamic, gpu_visible);

  EXPECT_TRUE(cpu_visible.empty());
  EXPECT_TRUE(gpu_visible.empty());
}

TEST_F(ColorizeRasterizerGpuTest, DynamicReturnOnSameSurfaceKeepsPoint)
{
  const std::vector<std::array<float, 3>> points = {{0.0F, 0.0F, 10.0F}};
  const std::vector<std::array<float, 3>> dynamic = {{0.0F, 0.0F, 9.6F}};
  std::vector<float> spacings = {0.05F};
  slam::ColorizeRasterizerConfig config;
  config.splat = false;

  auto gpu = slam::make_gpu_colorize_rasterizer(points, spacings, config);
  ASSERT_NE(gpu, nullptr);

  std::vector<slam::VisiblePoint> gpu_visible;
  gpu->visible_points(make_center_view(), dynamic, gpu_visible);
  ASSERT_EQ(gpu_visible.size(), 1U);
  EXPECT_EQ(gpu_visible[0].index, 0U);
}

}  // namespace
