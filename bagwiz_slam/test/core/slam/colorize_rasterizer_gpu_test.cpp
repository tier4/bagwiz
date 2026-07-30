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

  auto cpu = slam::make_cpu_colorize_rasterizer(points, spacings, {}, config);
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

  auto cpu = slam::make_cpu_colorize_rasterizer(points, spacings, {}, config);
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
    auto cpu = slam::make_cpu_colorize_rasterizer(points, spacings, {}, config);
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

TEST_F(ColorizeRasterizerGpuTest, EllipticalFootprintMatchesCpu)
{
  // The elliptical footprint runs in FP32 on the device against double on the
  // CPU, so a grazing road surface — the geometry the ellipse exists for,
  // where its determinant is smallest and cancellation most likely — must
  // still produce the same visible set on both backends. The samples are
  // staggered laterally so the off-diagonal covariance term is exercised too,
  // and spaced so no coverage decision sits within float epsilon of the
  // ellipse boundary: this test is about the two backends agreeing, not about
  // where exactly the boundary falls.
  const std::vector<std::array<float, 3>> points = {{-0.55F, 1.0F, 10.0F}, {-0.35F, 1.0F, 11.0F},
                                                    {-0.15F, 1.0F, 12.0F}, {0.15F, 1.0F, 13.0F},
                                                    {0.35F, 1.0F, 14.0F},  {0.55F, 1.0F, 15.0F}};
  const std::vector<std::array<float, 3>> normals(points.size(), {0.0F, 1.0F, 0.0F});
  const std::vector<float> spacings(points.size(), 0.5F);
  slam::ColorizeRasterizerConfig config;

  auto cpu = slam::make_cpu_colorize_rasterizer(points, spacings, normals, config);
  auto gpu = slam::make_gpu_colorize_rasterizer(points, spacings, normals, config);
  ASSERT_NE(gpu, nullptr);

  std::vector<slam::VisiblePoint> cpu_visible;
  std::vector<slam::VisiblePoint> gpu_visible;
  cpu->visible_points(make_center_view(), {}, cpu_visible);
  gpu->visible_points(make_center_view(), {}, gpu_visible);
  const auto oriented = sorted_indices(cpu_visible);
  EXPECT_EQ(oriented, sorted_indices(gpu_visible));

  auto isotropic_cpu = slam::make_cpu_colorize_rasterizer(points, spacings, {}, config);
  auto isotropic_gpu = slam::make_gpu_colorize_rasterizer(points, spacings, {}, config);
  ASSERT_NE(isotropic_gpu, nullptr);
  isotropic_cpu->visible_points(make_center_view(), {}, cpu_visible);
  isotropic_gpu->visible_points(make_center_view(), {}, gpu_visible);
  const auto isotropic = sorted_indices(cpu_visible);
  EXPECT_EQ(isotropic, sorted_indices(gpu_visible));

  EXPECT_GT(oriented.size(), isotropic.size());
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

  auto cpu = slam::make_cpu_colorize_rasterizer(points, spacings, {}, config);
  auto gpu = slam::make_gpu_colorize_rasterizer(points, spacings, {}, config);
  ASSERT_NE(gpu, nullptr);

  std::vector<slam::VisiblePoint> cpu_visible;
  std::vector<slam::VisiblePoint> gpu_visible;
  cpu->visible_points(make_center_view(), dynamic, cpu_visible);
  gpu->visible_points(make_center_view(), dynamic, gpu_visible);

  EXPECT_TRUE(cpu_visible.empty());
  EXPECT_TRUE(gpu_visible.empty());
}

// GPU/CPU agreement through a REAL distortion model: only a non-empty d
// exercises the forward distortion, the fixed-point undistortion (the FP32
// port loosened its convergence epsilon from 1e-10 to 1e-6), and the 1 px
// round-trip gate along a non-degenerate path. A 9x9 grid spanning the FOV
// out to `max_normalized_xy` must survive both backends identically and land
// within a small fraction of a pixel of the CPU double reference.
void expect_distorted_view_matches_cpu(
  const bagwiz::core::image::CameraInfo & camera, double max_normalized_xy)
{
  constexpr float kDepth = 5.0F;
  std::vector<std::array<float, 3>> points;
  for (int ix = -4; ix <= 4; ++ix) {
    for (int iy = -4; iy <= 4; ++iy) {
      const float nx = static_cast<float>(ix) / 4.0F * static_cast<float>(max_normalized_xy);
      const float ny = static_cast<float>(iy) / 4.0F * static_cast<float>(max_normalized_xy);
      points.push_back({nx * kDepth, ny * kDepth, kDepth});
    }
  }
  std::vector<float> spacings(points.size(), 0.05F);
  slam::ColorizeRasterizerConfig config;
  config.splat = false;

  slam::ColorizeView view = make_center_view();
  view.camera = camera;

  auto cpu = slam::make_cpu_colorize_rasterizer(points, spacings, {}, config);
  auto gpu = slam::make_gpu_colorize_rasterizer(points, spacings, {}, config);
  ASSERT_NE(gpu, nullptr);

  std::vector<slam::VisiblePoint> cpu_visible;
  std::vector<slam::VisiblePoint> gpu_visible;
  cpu->visible_points(view, {}, cpu_visible);
  gpu->visible_points(view, {}, gpu_visible);

  ASSERT_EQ(sorted_indices(cpu_visible), sorted_indices(gpu_visible));
  // The grid is sized to stay inside the image and pass the round-trip gate
  // on the CPU reference; a shrunken set would mean the test stopped
  // exercising the periphery.
  ASSERT_EQ(cpu_visible.size(), points.size());

  auto by_index = [](std::vector<slam::VisiblePoint> visible) {
    std::sort(visible.begin(), visible.end(), [](const auto & a, const auto & b) {
      return a.index < b.index;
    });
    return visible;
  };
  const auto cpu_sorted = by_index(cpu_visible);
  const auto gpu_sorted = by_index(gpu_visible);
  for (std::size_t i = 0; i < cpu_sorted.size(); ++i) {
    EXPECT_NEAR(gpu_sorted[i].u, cpu_sorted[i].u, 0.05) << "index " << cpu_sorted[i].index;
    EXPECT_NEAR(gpu_sorted[i].v, cpu_sorted[i].v, 0.05) << "index " << cpu_sorted[i].index;
    EXPECT_NEAR(gpu_sorted[i].depth, cpu_sorted[i].depth, 1e-3F);
  }
}

TEST_F(ColorizeRasterizerGpuTest, RationalPlumbBobDistortionMatchesCpu)
{
  // Realistic 8-coefficient rational plumb_bob (k1 k2 p1 p2 k3 k4 k5 k6):
  // moderate barrel distortion with mild tangential and rational terms, so
  // both the numerator and denominator polynomials and the tangential path
  // carry non-trivial values through the FP32 math.
  auto camera = make_pinhole();
  camera.d = {-0.2, 0.05, 1e-3, -5e-4, -0.004, -0.15, 0.03, -0.002};
  expect_distorted_view_matches_cpu(camera, 0.4);
}

TEST_F(ColorizeRasterizerGpuTest, EquidistantDistortionMatchesCpu)
{
  // Typical fisheye equidistant coefficients (k1..k4); the grid reaches a
  // normalized radius of ~0.7 (incidence ~35 deg), where theta polynomial
  // terms and the tan(theta) inversion are far from their small-angle
  // degeneracy.
  auto camera = make_pinhole();
  camera.distortion_model = "equidistant";
  camera.d = {-0.01, 0.02, -0.003, 4e-4};
  expect_distorted_view_matches_cpu(camera, 0.5);
}

TEST_F(ColorizeRasterizerGpuTest, FarFromOriginMatchesCpuWithinTolerance)
{
  // The device math runs in FP32 (the CPU reference in double), so agreement
  // is bounded by the float quantization of the world coordinates. Two
  // kilometers from the origin one float ulp is ~0.24 mm, i.e. ~5e-3 px at
  // f = 100 and z = 5 m: the visible sets must match exactly and the
  // projected pixels must agree to a small fraction of a pixel.
  const std::array<double, 3> center = {2000.0, -1500.0, 30.0};
  std::vector<std::array<float, 3>> points;
  for (const float gx : {-0.2F, -0.1F, 0.0F, 0.1F, 0.2F}) {
    for (const float gy : {-0.2F, -0.1F, 0.0F, 0.1F, 0.2F}) {
      points.push_back(
        {static_cast<float>(center[0]) + gx, static_cast<float>(center[1]) + gy,
         static_cast<float>(center[2]) + 5.0F});
    }
  }
  std::vector<float> spacings(points.size(), 0.05F);
  slam::ColorizeRasterizerConfig config;
  config.splat = false;

  slam::ColorizeView view;
  view.r_cam_world = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  view.t_cam_world = {-center[0], -center[1], -center[2]};
  view.camera = make_pinhole();
  view.width = 100;
  view.height = 100;

  auto cpu = slam::make_cpu_colorize_rasterizer(points, spacings, {}, config);
  auto gpu = slam::make_gpu_colorize_rasterizer(points, spacings, {}, config);
  ASSERT_NE(gpu, nullptr);

  std::vector<slam::VisiblePoint> cpu_visible;
  std::vector<slam::VisiblePoint> gpu_visible;
  cpu->visible_points(view, {}, cpu_visible);
  gpu->visible_points(view, {}, gpu_visible);

  ASSERT_EQ(sorted_indices(cpu_visible), sorted_indices(gpu_visible));
  ASSERT_EQ(cpu_visible.size(), points.size());

  auto by_index = [](const std::vector<slam::VisiblePoint> & visible) {
    std::vector<slam::VisiblePoint> sorted(visible);
    std::sort(sorted.begin(), sorted.end(), [](const auto & a, const auto & b) {
      return a.index < b.index;
    });
    return sorted;
  };
  const auto cpu_sorted = by_index(cpu_visible);
  const auto gpu_sorted = by_index(gpu_visible);
  for (std::size_t i = 0; i < cpu_sorted.size(); ++i) {
    EXPECT_NEAR(gpu_sorted[i].u, cpu_sorted[i].u, 0.05);
    EXPECT_NEAR(gpu_sorted[i].v, cpu_sorted[i].v, 0.05);
    EXPECT_NEAR(gpu_sorted[i].depth, cpu_sorted[i].depth, 1e-3F);
  }
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

}  // namespace
