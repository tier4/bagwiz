// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/cloud_transform.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <limits>
#include <vector>

namespace
{

using bagwiz::core::pointcloud::PointCloud2;
using bagwiz::core::pointcloud::PointFieldType;
using bagwiz::core::pointcloud::RigidTransform;
using bagwiz::core::pointcloud::transform_cloud_xyz;

// A cloud with x/y/z (FLOAT32) + a 1-byte intensity marker at offset 12,
// point_step 16. `pts` is a flat [x,y,z] list; `intensity` marks each point.
PointCloud2 make_cloud_f32(const std::vector<std::array<float, 3>> & pts, std::uint8_t intensity)
{
  PointCloud2 c;
  c.height = 1;
  c.width = static_cast<std::uint32_t>(pts.size());
  c.fields = {
    {"x", 0, PointFieldType::kFloat32, 1},
    {"y", 4, PointFieldType::kFloat32, 1},
    {"z", 8, PointFieldType::kFloat32, 1},
    {"intensity", 12, PointFieldType::kUint8, 1},
  };
  c.point_step = 16;
  c.row_step = 16 * c.width;
  c.data.assign(static_cast<std::size_t>(c.width) * 16, std::byte{0});
  for (std::size_t i = 0; i < pts.size(); ++i) {
    std::memcpy(c.data.data() + i * 16, pts[i].data(), sizeof(float) * 3);
    c.data[i * 16 + 12] = static_cast<std::byte>(intensity);
  }
  return c;
}

std::array<float, 3> point_at(const PointCloud2 & c, std::size_t i)
{
  std::array<float, 3> p{};
  std::memcpy(p.data(), c.data.data() + i * c.point_step, sizeof(float) * 3);
  return p;
}

}  // namespace

TEST(CloudTransform, PureTranslation)
{
  auto c = make_cloud_f32({{10.0f, 20.0f, 30.0f}}, 7);
  RigidTransform tf;  // identity rotation
  tf.translation = {1.0, 2.0, 3.0};
  ASSERT_TRUE(transform_cloud_xyz(c, tf).ok);
  const auto p = point_at(c, 0);
  EXPECT_FLOAT_EQ(p[0], 11.0f);
  EXPECT_FLOAT_EQ(p[1], 22.0f);
  EXPECT_FLOAT_EQ(p[2], 33.0f);
}

TEST(CloudTransform, BigEndianIsError)
{
  auto c = make_cloud_f32({{1.0f, 0, 0}}, 0);
  c.is_bigendian = true;
  EXPECT_FALSE(transform_cloud_xyz(c, RigidTransform{}).ok);
}

TEST(CloudTransform, NonFloatXyzIsError)
{
  PointCloud2 c;
  c.height = 1;
  c.width = 1;
  c.fields = {
    {"x", 0, PointFieldType::kInt32, 1},
    {"y", 4, PointFieldType::kInt32, 1},
    {"z", 8, PointFieldType::kInt32, 1},
  };
  c.point_step = 12;
  c.data.assign(12, std::byte{0});
  EXPECT_FALSE(transform_cloud_xyz(c, RigidTransform{}).ok);
}

TEST(CloudTransform, PointStepZeroIsError)
{
  auto c = make_cloud_f32({{1.0f, 0, 0}}, 0);
  c.point_step = 0;
  EXPECT_FALSE(transform_cloud_xyz(c, RigidTransform{}).ok);
}

// A field whose offset + size exceeds point_step would make the per-point loop
// read/write past the data buffer; it must be rejected before that happens.
TEST(CloudTransform, FieldPastPointStepIsError)
{
  PointCloud2 c;
  c.height = 1;
  c.width = 1;
  c.fields = {
    {"x", 0, PointFieldType::kFloat32, 1},
    {"y", 4, PointFieldType::kFloat32, 1},
    {"z", 12, PointFieldType::kFloat32, 1},  // 12 + 4 = 16 > point_step 12
  };
  c.point_step = 12;
  c.data.assign(12, std::byte{0});
  const auto r = transform_cloud_xyz(c, RigidTransform{});
  EXPECT_FALSE(r.ok);
  EXPECT_FALSE(r.error.empty());
}

TEST(CloudTransform, PureRotationAboutZ)
{
  auto c = make_cloud_f32({{1.0f, 0.0f, 0.0f}}, 0);
  RigidTransform tf;
  // +90 deg about z: (x,y,z) -> (-y, x, z)
  tf.rotation = {0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  ASSERT_TRUE(transform_cloud_xyz(c, tf).ok);
  const auto p = point_at(c, 0);
  EXPECT_NEAR(p[0], 0.0f, 1e-5);
  EXPECT_NEAR(p[1], 1.0f, 1e-5);
  EXPECT_NEAR(p[2], 0.0f, 1e-5);
}

TEST(CloudTransform, PreservesNonXyzFieldsAndNonFinitePoints)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  auto c = make_cloud_f32({{1.0f, 1.0f, 1.0f}, {nan, 2.0f, 3.0f}}, 42);
  RigidTransform tf;
  tf.translation = {5.0, 0.0, 0.0};
  ASSERT_TRUE(transform_cloud_xyz(c, tf).ok);

  // Finite point moved; NaN point left verbatim.
  const auto p0 = point_at(c, 0);
  EXPECT_FLOAT_EQ(p0[0], 6.0f);
  const auto p1 = point_at(c, 1);
  EXPECT_TRUE(std::isnan(p1[0]));
  EXPECT_FLOAT_EQ(p1[1], 2.0f);
  EXPECT_FLOAT_EQ(p1[2], 3.0f);

  // Intensity markers untouched for both points.
  EXPECT_EQ(std::to_integer<int>(c.data[12]), 42);
  EXPECT_EQ(std::to_integer<int>(c.data[16 + 12]), 42);
}

TEST(CloudTransform, IdentityIsNoOp)
{
  auto c = make_cloud_f32({{1.5f, 2.5f, 3.5f}}, 1);
  const auto before = c.data;
  ASSERT_TRUE(transform_cloud_xyz(c, RigidTransform{}).ok);
  EXPECT_EQ(c.data, before);
}

TEST(CloudTransform, Float64Coordinates)
{
  PointCloud2 c;
  c.height = 1;
  c.width = 1;
  c.fields = {
    {"x", 0, PointFieldType::kFloat64, 1},
    {"y", 8, PointFieldType::kFloat64, 1},
    {"z", 16, PointFieldType::kFloat64, 1},
  };
  c.point_step = 24;
  c.row_step = 24;
  c.data.assign(24, std::byte{0});
  const double pts[3] = {1.0, 2.0, 3.0};
  std::memcpy(c.data.data(), pts, sizeof(pts));

  RigidTransform tf;
  tf.translation = {10.0, 20.0, 30.0};
  ASSERT_TRUE(transform_cloud_xyz(c, tf).ok);
  double out[3] = {0, 0, 0};
  std::memcpy(out, c.data.data(), sizeof(out));
  EXPECT_DOUBLE_EQ(out[0], 11.0);
  EXPECT_DOUBLE_EQ(out[1], 22.0);
  EXPECT_DOUBLE_EQ(out[2], 33.0);
}

TEST(CloudTransform, MissingXyzIsError)
{
  PointCloud2 c;
  c.height = 1;
  c.width = 1;
  c.fields = {{"x", 0, PointFieldType::kFloat32, 1}, {"y", 4, PointFieldType::kFloat32, 1}};
  c.point_step = 8;
  c.data.assign(8, std::byte{0});
  const auto result = transform_cloud_xyz(c, RigidTransform{});
  EXPECT_FALSE(result.ok);
  EXPECT_FALSE(result.error.empty());
}
