// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/projector.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

namespace
{

using bagwiz::core::image::CameraInfo;
using bagwiz::core::pointcloud::PointCloud2;
using bagwiz::core::pointcloud::PointCloudProperty;
using bagwiz::core::pointcloud::PointField;
using bagwiz::core::pointcloud::PointFieldType;
using bagwiz::core::pointcloud::project_pointcloud;

PointCloud2 make_xy_cloud(const std::vector<std::array<float, 2>> & points)
{
  PointCloud2 cloud;
  cloud.height = 1;
  cloud.width = static_cast<std::uint32_t>(points.size());
  cloud.fields = {
    PointField{"x", 0, PointFieldType::kFloat32, 1},
    PointField{"y", 4, PointFieldType::kFloat32, 1},
  };
  cloud.point_step = 8;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.data.resize(cloud.row_step);
  for (std::size_t i = 0; i < points.size(); ++i) {
    std::memcpy(cloud.data.data() + i * cloud.point_step, points[i].data(), cloud.point_step);
  }
  cloud.is_dense = true;
  return cloud;
}

PointCloud2 make_xyz_cloud(const std::vector<std::array<float, 3>> & points)
{
  PointCloud2 cloud;
  cloud.height = 1;
  cloud.width = static_cast<std::uint32_t>(points.size());
  cloud.fields = {
    PointField{"x", 0, PointFieldType::kFloat32, 1},
    PointField{"y", 4, PointFieldType::kFloat32, 1},
    PointField{"z", 8, PointFieldType::kFloat32, 1},
  };
  cloud.point_step = 12;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.data.resize(cloud.row_step);
  for (std::size_t i = 0; i < points.size(); ++i) {
    std::memcpy(cloud.data.data() + i * cloud.point_step, points[i].data(), cloud.point_step);
  }
  cloud.is_dense = true;
  return cloud;
}

PointCloud2 make_xyz_intensity_cloud(const std::vector<std::array<float, 4>> & points)
{
  PointCloud2 cloud;
  cloud.height = 1;
  cloud.width = static_cast<std::uint32_t>(points.size());
  cloud.fields = {
    PointField{"x", 0, PointFieldType::kFloat32, 1},
    PointField{"y", 4, PointFieldType::kFloat32, 1},
    PointField{"z", 8, PointFieldType::kFloat32, 1},
    PointField{"intensity", 12, PointFieldType::kFloat32, 1},
  };
  cloud.point_step = 16;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.data.resize(cloud.row_step);
  for (std::size_t i = 0; i < points.size(); ++i) {
    std::memcpy(cloud.data.data() + i * cloud.point_step, points[i].data(), cloud.point_step);
  }
  cloud.is_dense = true;
  return cloud;
}

CameraInfo make_pinhole_camera(
  double fx, double fy, double cx, double cy, std::uint32_t width, std::uint32_t height)
{
  CameraInfo info;
  info.width = width;
  info.height = height;
  info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
  return info;
}

CameraInfo make_pinhole_camera_with_p(
  double fx, double fy, double cx, double cy, std::uint32_t width, std::uint32_t height,
  double p_fx, double p_fy, double p_cx, double p_cy)
{
  CameraInfo info;
  info.width = width;
  info.height = height;
  info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
  info.p = {p_fx, 0.0, p_cx, 0.0, 0.0, p_fy, p_cy, 0.0, 0.0, 0.0, 1.0, 0.0};
  return info;
}

std::array<double, 16> identity_transform()
{
  return {
    1.0, 0.0, 0.0, 0.0,  // column 0
    0.0, 1.0, 0.0, 0.0,  // column 1
    0.0, 0.0, 1.0, 0.0,  // column 2
    0.0, 0.0, 0.0, 1.0,  // column 3
  };
}

std::array<double, 16> translate_z_transform(double delta_z)
{
  return {
    1.0, 0.0, 0.0,     0.0,  // column 0
    0.0, 1.0, 0.0,     0.0,  // column 1
    0.0, 0.0, 1.0,     0.0,  // column 2
    0.0, 0.0, delta_z, 1.0   // column 3 (translation)
  };
}

}  // namespace

TEST(Projector, FrontPointProjectsToImageCenter)
{
  const auto cloud = make_xyz_cloud({{0.0f, 0.0f, 5.0f}});
  const auto camera = make_pinhole_camera(100.0, 100.0, 320.0, 240.0, 640, 480);

  const auto result = project_pointcloud(
    cloud, camera, identity_transform(), 640, 480, PointCloudProperty::kDistance);

  ASSERT_TRUE(result.ok());
  ASSERT_EQ(result.points.size(), 1u);
  EXPECT_EQ(result.points[0].u, 320);
  EXPECT_EQ(result.points[0].v, 240);
  EXPECT_FLOAT_EQ(result.points[0].depth, 5.0f);
  EXPECT_FLOAT_EQ(result.points[0].value, 5.0f);
}

TEST(Projector, DropsPointBehindCamera)
{
  const auto cloud = make_xyz_cloud({{0.0f, 0.0f, -5.0f}});
  const auto camera = make_pinhole_camera(100.0, 100.0, 320.0, 240.0, 640, 480);

  const auto result = project_pointcloud(
    cloud, camera, identity_transform(), 640, 480, PointCloudProperty::kDistance);

  ASSERT_TRUE(result.ok());
  EXPECT_TRUE(result.points.empty());
}

TEST(Projector, DropsPointOutsideImageBounds)
{
  // x=100 projects far to the right of a 640-pixel image.
  const auto cloud = make_xyz_cloud({{100.0f, 0.0f, 5.0f}});
  const auto camera = make_pinhole_camera(100.0, 100.0, 320.0, 240.0, 640, 480);

  const auto result = project_pointcloud(
    cloud, camera, identity_transform(), 640, 480, PointCloudProperty::kDistance);

  ASSERT_TRUE(result.ok());
  EXPECT_TRUE(result.points.empty());
}

TEST(Projector, MissingXYZFieldReturnsError)
{
  const auto cloud = make_xy_cloud({{1.0f, 2.0f}});
  const auto camera = make_pinhole_camera(100.0, 100.0, 320.0, 240.0, 640, 480);

  const auto result = project_pointcloud(
    cloud, camera, identity_transform(), 640, 480, PointCloudProperty::kDistance);

  EXPECT_FALSE(result.ok());
  EXPECT_FALSE(result.error.empty());
  EXPECT_TRUE(result.points.empty());
}

TEST(Projector, MissingIntensityFieldReturnsError)
{
  const auto cloud = make_xyz_cloud({{0.0f, 0.0f, 5.0f}});
  const auto camera = make_pinhole_camera(100.0, 100.0, 320.0, 240.0, 640, 480);

  const auto result = project_pointcloud(
    cloud, camera, identity_transform(), 640, 480, PointCloudProperty::kIntensity);

  EXPECT_FALSE(result.ok());
  EXPECT_FALSE(result.error.empty());
  EXPECT_TRUE(result.points.empty());
}

TEST(Projector, IntensityPropertyReadsIntensityField)
{
  const auto cloud = make_xyz_intensity_cloud({{1.0f, 2.0f, 5.0f, 42.0f}});
  const auto camera = make_pinhole_camera(100.0, 100.0, 320.0, 240.0, 640, 480);

  const auto result = project_pointcloud(
    cloud, camera, identity_transform(), 640, 480, PointCloudProperty::kIntensity);

  ASSERT_TRUE(result.ok());
  ASSERT_EQ(result.points.size(), 1u);
  EXPECT_FLOAT_EQ(result.points[0].value, 42.0f);
}

TEST(Projector, PropertyXYZReturnsRawCoordinates)
{
  const auto cloud = make_xyz_cloud({{1.0f, 2.0f, 3.0f}});
  const auto camera = make_pinhole_camera(100.0, 100.0, 320.0, 240.0, 640, 480);

  const auto rx =
    project_pointcloud(cloud, camera, identity_transform(), 640, 480, PointCloudProperty::kX);
  const auto ry =
    project_pointcloud(cloud, camera, identity_transform(), 640, 480, PointCloudProperty::kY);
  const auto rz =
    project_pointcloud(cloud, camera, identity_transform(), 640, 480, PointCloudProperty::kZ);

  ASSERT_TRUE(rx.ok());
  ASSERT_TRUE(ry.ok());
  ASSERT_TRUE(rz.ok());
  ASSERT_EQ(rx.points.size(), 1u);
  ASSERT_EQ(ry.points.size(), 1u);
  ASSERT_EQ(rz.points.size(), 1u);
  EXPECT_FLOAT_EQ(rx.points[0].value, 1.0f);
  EXPECT_FLOAT_EQ(ry.points[0].value, 2.0f);
  EXPECT_FLOAT_EQ(rz.points[0].value, 3.0f);
}

TEST(Projector, TranslationTransformProjectsCorrectly)
{
  // Point (1, 0, 4) with a z-translation of +1 moves it to (1, 0, 5).
  // Without translation it would project to u=345; with translation u=340.
  const auto cloud = make_xyz_cloud({{1.0f, 0.0f, 4.0f}});
  const auto camera = make_pinhole_camera(100.0, 100.0, 320.0, 240.0, 640, 480);

  const auto result = project_pointcloud(
    cloud, camera, translate_z_transform(1.0), 640, 480, PointCloudProperty::kDistance);

  ASSERT_TRUE(result.ok());
  ASSERT_EQ(result.points.size(), 1u);
  EXPECT_EQ(result.points[0].u, 340);
  EXPECT_EQ(result.points[0].v, 240);
  EXPECT_FLOAT_EQ(result.points[0].depth, 5.0f);
  EXPECT_FLOAT_EQ(result.points[0].value, std::sqrt(1.0f + 0.0f + 16.0f));
}

TEST(Projector, UseRectifiedSelectsProjectionMatrix)
{
  // A camera whose k and p differ. With use_rectified=true the projection uses p.
  const auto cloud = make_xyz_cloud({{0.0f, 0.0f, 5.0f}});
  const auto camera =
    make_pinhole_camera_with_p(100.0, 100.0, 320.0, 240.0, 640, 480, 200.0, 200.0, 160.0, 120.0);

  const auto raw = project_pointcloud(
    cloud, camera, identity_transform(), 640, 480, PointCloudProperty::kDistance, false);
  const auto rectified = project_pointcloud(
    cloud, camera, identity_transform(), 640, 480, PointCloudProperty::kDistance, true);

  ASSERT_TRUE(raw.ok());
  ASSERT_TRUE(rectified.ok());
  ASSERT_EQ(raw.points.size(), 1u);
  ASSERT_EQ(rectified.points.size(), 1u);
  EXPECT_EQ(raw.points[0].u, 320);
  EXPECT_EQ(raw.points[0].v, 240);
  EXPECT_EQ(rectified.points[0].u, 160);
  EXPECT_EQ(rectified.points[0].v, 120);
}
