// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/project.hpp"

#include <gtest/gtest.h>

#include <tf2/LinearMath/Transform.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <tuple>
#include <vector>

namespace
{
using bagwiz::core::camera::CameraInfo;
using bagwiz::core::color::ColorMapName;
using bagwiz::core::pointcloud::ColorBy;
using bagwiz::core::pointcloud::PointCloudView;
using bagwiz::core::pointcloud::project_point_cloud;
using bagwiz::core::pointcloud::ProjectedPoint;

struct SimpleCloud
{
  std::vector<std::byte> data;
  PointCloudView view;
};

void append_f32(std::vector<std::byte> & dst, float v)
{
  std::array<std::byte, 4> bytes{};
  std::memcpy(bytes.data(), &v, sizeof(v));
  dst.insert(dst.end(), bytes.begin(), bytes.end());
}

SimpleCloud make_simple_cloud(
  const std::vector<std::array<float, 4>> & points, bool with_intensity = false)
{
  PointCloudView view;
  view.width = static_cast<std::uint32_t>(points.size());
  view.height = 1;
  view.point_step = with_intensity ? 16U : 12U;
  view.x_offset = 0U;
  view.y_offset = 4U;
  view.z_offset = 8U;
  if (with_intensity) {
    view.intensity_offset = 12U;
    view.intensity_datatype = 7U;  // FLOAT32
  } else {
    view.intensity_offset = std::nullopt;
    view.intensity_datatype = 0U;
  }
  view.is_bigendian = false;
  view.is_dense = true;

  std::vector<std::byte> data;
  data.reserve(points.size() * view.point_step);
  for (const auto & p : points) {
    append_f32(data, p[0]);  // x
    append_f32(data, p[1]);  // y
    append_f32(data, p[2]);  // z
    if (with_intensity) {
      append_f32(data, p[3]);  // intensity
    }
  }
  view.data = data;
  return SimpleCloud{std::move(data), view};
}

CameraInfo make_camera()
{
  CameraInfo cam;
  cam.K = {500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0};
  cam.width = 640;
  cam.height = 480;
  return cam;
}

TEST(ProjectTest, ProjectsKnownPoints)
{
  const auto cam = make_camera();
  tf2::Transform identity;
  identity.setIdentity();

  const auto cloud = make_simple_cloud({{0.0f, 0.0f, 10.0f, 0.0f}});
  const auto result = project_point_cloud(cloud.view, cam, identity, ColorBy::kZ, ColorMapName::kJet);

  ASSERT_TRUE(result.ok) << result.error;
  ASSERT_EQ(result.points.size(), 1u);
  EXPECT_EQ(result.points[0].u, 320);
  EXPECT_EQ(result.points[0].v, 240);
  EXPECT_FLOAT_EQ(result.points[0].depth, 10.0f);
}

TEST(ProjectTest, DropsPointsBehindCamera)
{
  const auto cam = make_camera();
  tf2::Transform identity;
  identity.setIdentity();

  const auto cloud = make_simple_cloud({{0.0f, 0.0f, -5.0f, 0.0f}});
  const auto result = project_point_cloud(cloud.view, cam, identity, ColorBy::kZ, ColorMapName::kJet);

  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_TRUE(result.points.empty());
}

TEST(ProjectTest, DropsPointsAtZeroDepth)
{
  const auto cam = make_camera();
  tf2::Transform identity;
  identity.setIdentity();

  const auto cloud = make_simple_cloud({{0.0f, 0.0f, 0.0f, 0.0f}});
  const auto result = project_point_cloud(cloud.view, cam, identity, ColorBy::kZ, ColorMapName::kJet);

  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_TRUE(result.points.empty());
}

TEST(ProjectTest, DropsOutOfImagePoints)
{
  const auto cam = make_camera();
  tf2::Transform identity;
  identity.setIdentity();

  // Far to the right; u would exceed image width.
  const auto cloud = make_simple_cloud({{20.0f, 0.0f, 10.0f, 0.0f}});
  const auto result = project_point_cloud(cloud.view, cam, identity, ColorBy::kZ, ColorMapName::kJet);

  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_TRUE(result.points.empty());
}

TEST(ProjectTest, DifferentDepthsProduceDifferentColors)
{
  const auto cam = make_camera();
  tf2::Transform identity;
  identity.setIdentity();

  const auto cloud = make_simple_cloud({{0.0f, 0.0f, 5.0f, 0.0f}, {0.0f, 0.0f, 15.0f, 0.0f}});
  const auto result = project_point_cloud(cloud.view, cam, identity, ColorBy::kZ, ColorMapName::kJet);

  ASSERT_TRUE(result.ok) << result.error;
  ASSERT_EQ(result.points.size(), 2u);
  EXPECT_NE(result.points[0].rgb.r, result.points[1].rgb.r);
}

TEST(ProjectTest, SortsByDepthFrontToBack)
{
  const auto cam = make_camera();
  tf2::Transform identity;
  identity.setIdentity();

  const auto cloud = make_simple_cloud({
    {0.0f, 0.0f, 20.0f, 0.0f},
    {0.0f, 0.0f, 5.0f, 0.0f},
    {0.0f, 0.0f, 15.0f, 0.0f},
  });
  const auto result = project_point_cloud(cloud.view, cam, identity, ColorBy::kZ, ColorMapName::kJet);

  ASSERT_TRUE(result.ok) << result.error;
  ASSERT_EQ(result.points.size(), 3u);
  EXPECT_FLOAT_EQ(result.points[0].depth, 5.0f);
  EXPECT_FLOAT_EQ(result.points[1].depth, 15.0f);
  EXPECT_FLOAT_EQ(result.points[2].depth, 20.0f);
}

TEST(ProjectTest, IntensityMissingReturnsError)
{
  const auto cam = make_camera();
  tf2::Transform identity;
  identity.setIdentity();

  const auto cloud = make_simple_cloud({{0.0f, 0.0f, 10.0f, 0.0f}});
  const auto result = project_point_cloud(cloud.view, cam, identity, ColorBy::kIntensity, ColorMapName::kJet);

  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.error.find("intensity field missing"), std::string::npos);
  EXPECT_TRUE(result.points.empty());
}

TEST(ProjectTest, IntensityPresentProducesColors)
{
  const auto cam = make_camera();
  tf2::Transform identity;
  identity.setIdentity();

  const auto cloud = make_simple_cloud({{0.0f, 0.0f, 10.0f, 0.25f}}, true);
  const auto result = project_point_cloud(cloud.view, cam, identity, ColorBy::kIntensity, ColorMapName::kJet);

  ASSERT_TRUE(result.ok) << result.error;
  ASSERT_EQ(result.points.size(), 1u);
  EXPECT_EQ(result.points[0].u, 320);
  EXPECT_EQ(result.points[0].v, 240);
}

TEST(ProjectTest, ColorByDistanceUsesCameraFrameDistance)
{
  const auto cam = make_camera();
  tf2::Transform identity;
  identity.setIdentity();

  // A point away from the optical axis but still inside the image; the scalar
  // is the full camera-frame distance, not just z.
  const auto cloud = make_simple_cloud({{0.3f, 0.4f, 5.0f, 0.0f}});
  const auto result = project_point_cloud(cloud.view, cam, identity, ColorBy::kDistance, ColorMapName::kJet);

  ASSERT_TRUE(result.ok) << result.error;
  ASSERT_EQ(result.points.size(), 1u);
  EXPECT_EQ(result.points[0].u, 350);
  EXPECT_EQ(result.points[0].v, 280);
}

}  // namespace
