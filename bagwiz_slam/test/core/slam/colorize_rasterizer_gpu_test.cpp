// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/image/sampling.hpp"
#include "bagwiz/core/slam/colorize_rasterizer.hpp"
#include "bagwiz/core/slam/colorize_rasterizer_gpu.hpp"
#include "bagwiz/core/slam/colorize_weight.hpp"

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
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
  auto gpu = slam::make_gpu_colorize_rasterizer(points, spacings, {}, config);
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
  auto gpu = slam::make_gpu_colorize_rasterizer(points, spacings, {}, config);
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
    auto gpu = slam::make_gpu_colorize_rasterizer(points, spacings, {}, config);
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
  auto gpu = slam::make_gpu_colorize_rasterizer(points, spacings, {}, config);
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

  auto gpu = slam::make_gpu_colorize_rasterizer(points, spacings, {}, config);
  ASSERT_NE(gpu, nullptr);

  std::vector<slam::VisiblePoint> gpu_visible;
  gpu->visible_points(make_center_view(), dynamic, gpu_visible);
  ASSERT_EQ(gpu_visible.size(), 1U);
  EXPECT_EQ(gpu_visible[0].index, 0U);
}

// Builds a 4x4 red+green checkerboard BGR24 raster centered at (50, 50).
std::vector<std::byte> make_checkerboard_raster()
{
  constexpr std::uint32_t width = 100;
  constexpr std::uint32_t height = 100;
  std::vector<std::byte> raster(static_cast<std::size_t>(width) * height * 3);
  for (std::uint32_t row = 0; row < height; ++row) {
    for (std::uint32_t col = 0; col < width; ++col) {
      const std::size_t idx = (static_cast<std::size_t>(row) * width + col) * 3;
      const bool red = (col / 4 + row / 4) % 2 == 0;
      raster[idx + 0] = static_cast<std::byte>(0);      // b
      raster[idx + 1] = static_cast<std::byte>(red ? 0 : 255);  // g
      raster[idx + 2] = static_cast<std::byte>(red ? 255 : 0);  // r
    }
  }
  return raster;
}

TEST_F(ColorizeRasterizerGpuTest, SampleObservationsMatchCpu)
{
  const std::vector<std::array<float, 3>> points = {
    {-0.05F, -0.05F, 5.0F},
    {0.05F, -0.05F, 5.0F},
    {-0.05F, 0.05F, 5.0F},
    {0.05F, 0.05F, 5.0F}};
  const std::vector<float> spacings(points.size(), 0.05F);
  const std::vector<std::array<float, 3>> normals(points.size(), {0.0F, 0.0F, 1.0F});
  slam::ColorizeRasterizerConfig config;
  config.splat = false;

  auto gpu = slam::make_gpu_colorize_rasterizer(points, spacings, normals, config);
  ASSERT_NE(gpu, nullptr);
  EXPECT_TRUE(gpu->can_sample());

  const auto raster = make_checkerboard_raster();
  const slam::ObservationWeightParams params{15.0, 10.0, 16.0};
  std::vector<slam::ColorizeObservation> gpu_obs;
  gpu->sample_observations(
    make_center_view(), std::span<const std::byte>(raster), {}, {0.0, 0.0, 0.0}, normals, true,
    params, 1e-3, gpu_obs);

  // CPU reference: visible_points + weight_and_sample equivalent.
  auto cpu = slam::make_cpu_colorize_rasterizer(points, spacings, config);
  std::vector<slam::VisiblePoint> cpu_visible;
  cpu->visible_points(make_center_view(), {}, cpu_visible);

  auto by_index = [](const slam::ColorizeObservation & a, const slam::ColorizeObservation & b) {
    return a.index < b.index;
  };
  std::sort(gpu_obs.begin(), gpu_obs.end(), by_index);

  ASSERT_EQ(gpu_obs.size(), cpu_visible.size());
  for (std::size_t i = 0; i < cpu_visible.size(); ++i) {
    const auto & vp = cpu_visible[i];
    const auto it = std::lower_bound(
      gpu_obs.begin(), gpu_obs.end(), slam::ColorizeObservation{vp.index, 0, 0, 0, 0}, by_index);
    ASSERT_NE(it, gpu_obs.end());
    ASSERT_EQ(it->index, vp.index);

    const auto sample = bagwiz::core::image::bilinear_sample_bgr(
      std::span<const std::byte>(raster), make_center_view().width, make_center_view().height, vp.u,
      vp.v);
    EXPECT_NEAR(it->r, sample[2], 1e-3);
    EXPECT_NEAR(it->g, sample[1], 1e-3);
    EXPECT_NEAR(it->b, sample[0], 1e-3);

    const double w = bagwiz::core::slam::compute_observation_weight(
      vp, points, normals, {0.0, 0.0, 0.0}, std::span<const std::byte>(raster),
      make_center_view().width, make_center_view().height, params);
    EXPECT_NEAR(it->weight, w, 1e-3);
  }
}

TEST_F(ColorizeRasterizerGpuTest, SampleObservationsNoWeights)
{
  const std::vector<std::array<float, 3>> points = {{0.0F, 0.0F, 5.0F}};
  const std::vector<float> spacings = {0.05F};
  slam::ColorizeRasterizerConfig config;
  config.splat = false;

  auto gpu = slam::make_gpu_colorize_rasterizer(points, spacings, {}, config);
  ASSERT_NE(gpu, nullptr);

  const auto raster = make_checkerboard_raster();
  const slam::ObservationWeightParams params{15.0, 10.0, 16.0};
  std::vector<slam::ColorizeObservation> gpu_obs;
  gpu->sample_observations(
    make_center_view(), std::span<const std::byte>(raster), {}, {0.0, 0.0, 0.0}, {}, false, params,
    1e-3, gpu_obs);

  ASSERT_EQ(gpu_obs.size(), 1U);
  EXPECT_EQ(gpu_obs[0].weight, 1.0);
}

}  // namespace
