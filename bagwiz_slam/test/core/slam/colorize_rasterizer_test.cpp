// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/colorize_rasterizer.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
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

std::vector<std::uint32_t> run(
  const std::vector<std::array<float, 3>> & points, const std::vector<float> & spacings,
  const std::vector<std::array<float, 3>> & normals, const slam::ColorizeRasterizerConfig & config)
{
  auto rasterizer = slam::make_cpu_colorize_rasterizer(points, spacings, normals, config);
  std::vector<slam::VisiblePoint> visible;
  rasterizer->visible_points(make_center_view(), {}, visible);
  std::vector<std::uint32_t> indices;
  indices.reserve(visible.size());
  for (const auto & p : visible) {
    indices.push_back(p.index);
  }
  std::sort(indices.begin(), indices.end());
  return indices;
}

// Three samples of a road surface running away from the camera at 1 m steps,
// one meter below it: the grazing geometry a vehicle-mounted camera spends
// most of its frames looking at. They project a pixel or less apart
// vertically (v = 60.0, 59.1, 58.3) while their depths differ by a full
// meter, far more than the depth tolerance — so any footprint that reaches a
// neighbour's pixel culls that neighbour.
const std::vector<std::array<float, 3>> kRoadPoints = {
  {0.0F, 1.0F, 10.0F}, {0.0F, 1.0F, 11.0F}, {0.0F, 1.0F, 12.0F}};
// Surface normal of that road, perpendicular to the view direction.
const std::vector<std::array<float, 3>> kRoadNormals = {
  {0.0F, 1.0F, 0.0F}, {0.0F, 1.0F, 0.0F}, {0.0F, 1.0F, 0.0F}};
const std::vector<float> kRoadSpacings = {0.5F, 0.5F, 0.5F};

TEST(CpuColorizeRasterizer, CircularFootprintOverCullsGrazingGeometry)
{
  // The behavior the elliptical footprint replaces, still reachable whenever
  // no normal is available: each point's isotropic disc (2.1 - 2.5 px here)
  // reaches its neighbours' pixels and stamps its own much nearer depth
  // there, so only the closest sample of the surface survives.
  slam::ColorizeRasterizerConfig config;
  config.num_threads = 1;

  EXPECT_EQ(run(kRoadPoints, kRoadSpacings, {}, config), (std::vector<std::uint32_t>{0}));
}

TEST(CpuColorizeRasterizer, EllipticalFootprintKeepsGrazingGeometryVisible)
{
  // With the surfel's orientation known, the same discs foreshorten to
  // 0.2 px tall slivers along the tilt direction — they still span the
  // direction the samples are spread out in, but stop short of the
  // neighbouring sample they used to swallow. All three survive.
  slam::ColorizeRasterizerConfig config;
  config.num_threads = 1;

  EXPECT_EQ(
    run(kRoadPoints, kRoadSpacings, kRoadNormals, config), (std::vector<std::uint32_t>{0, 1, 2}));
}

TEST(CpuColorizeRasterizer, FrontoParallelNormalsReproduceTheIsotropicDisc)
{
  // A surfel facing the camera projects to a circle, so supplying normals for
  // a wall must not change what a wall occludes: four sparse points at z = 5
  // land diagonally around pixel (50, 50) and their splats close that hole
  // against the far point behind it, normals or not.
  const std::vector<std::array<float, 3>> points = {
    {-0.05F, -0.05F, 5.0F},
    {0.05F, -0.05F, 5.0F},
    {-0.05F, 0.05F, 5.0F},
    {0.05F, 0.05F, 5.0F},
    {0.0F, 0.0F, 10.0F}};
  const std::vector<float> spacings(points.size(), 0.15F);
  const std::vector<std::array<float, 3>> normals(points.size(), {0.0F, 0.0F, 1.0F});
  slam::ColorizeRasterizerConfig config;
  config.num_threads = 1;

  const std::vector<std::uint32_t> expected = {0, 1, 2, 3};  // far point occluded
  EXPECT_EQ(run(points, spacings, {}, config), expected);
  EXPECT_EQ(run(points, spacings, normals, config), expected);
}

TEST(CpuColorizeRasterizer, EdgeOnPointStillOccludesWhatIsDirectlyBehindIt)
{
  // The footprint of an exactly edge-on surfel collapses, but the center
  // pixel is always written: a point sharing the occluder's pixel is still
  // rejected, so the degenerate case loses coverage without losing the
  // z-buffer itself.
  const std::vector<std::array<float, 3>> points = {{0.0F, 0.0F, 5.0F}, {0.0F, 0.0F, 10.0F}};
  const std::vector<float> spacings(points.size(), 0.5F);
  const std::vector<std::array<float, 3>> normals(points.size(), {1.0F, 0.0F, 0.0F});
  slam::ColorizeRasterizerConfig config;
  config.num_threads = 1;

  EXPECT_EQ(run(points, spacings, normals, config), (std::vector<std::uint32_t>{0}));
}

TEST(CpuColorizeRasterizer, ZeroNormalSentinelFallsBackToTheIsotropicDisc)
{
  // The geometry pre-pass emits {0, 0, 0} where a neighborhood was too small
  // or too degenerate to fit a plane. Those points must keep the isotropic
  // footprint rather than collapsing as if they were edge-on.
  const std::vector<std::array<float, 3>> normals(kRoadPoints.size(), {0.0F, 0.0F, 0.0F});
  slam::ColorizeRasterizerConfig config;
  config.num_threads = 1;

  EXPECT_EQ(run(kRoadPoints, kRoadSpacings, normals, config), (std::vector<std::uint32_t>{0}));
}

TEST(CpuColorizeRasterizer, MismatchedNormalsSpanIsIgnored)
{
  // A caller that hands over a span of the wrong length gets the isotropic
  // behavior, matching how a mismatched `spacings` span disables the splat
  // rather than indexing out of range.
  const std::vector<std::array<float, 3>> normals = {{0.0F, 1.0F, 0.0F}};
  slam::ColorizeRasterizerConfig config;
  config.num_threads = 1;

  EXPECT_EQ(run(kRoadPoints, kRoadSpacings, normals, config), (std::vector<std::uint32_t>{0}));
}

TEST(CpuColorizeRasterizer, SplatDisabledKeepsEveryPointRegardlessOfNormals)
{
  // Turning the splat off leaves each point writing only its own pixel, so
  // the road samples never touch each other whether or not normals are given.
  slam::ColorizeRasterizerConfig config;
  config.num_threads = 1;
  config.splat = false;

  const std::vector<std::uint32_t> expected = {0, 1, 2};
  EXPECT_EQ(run(kRoadPoints, kRoadSpacings, kRoadNormals, config), expected);
  EXPECT_EQ(run(kRoadPoints, kRoadSpacings, {}, config), expected);
}

TEST(CpuColorizeRasterizer, MultithreadedRunMatchesSingleThread)
{
  // The depth buffer is reduced with an order-independent atomic min and the
  // per-chunk candidate lists merge in chunk order, so the elliptical
  // footprint must not have introduced any thread-count dependence.
  std::vector<std::array<float, 3>> points;
  std::vector<std::array<float, 3>> normals;
  std::vector<float> spacings;
  for (int i = 0; i < 400; ++i) {
    const float t = static_cast<float>(i) * 0.05F;
    points.push_back({t * 0.1F - 1.0F, 1.0F, 6.0F + t});
    normals.push_back({0.0F, 1.0F, 0.0F});
    spacings.push_back(0.3F);
  }

  slam::ColorizeRasterizerConfig serial;
  serial.num_threads = 1;
  slam::ColorizeRasterizerConfig parallel;
  parallel.num_threads = 8;

  EXPECT_EQ(run(points, spacings, normals, serial), run(points, spacings, normals, parallel));
}

}  // namespace
