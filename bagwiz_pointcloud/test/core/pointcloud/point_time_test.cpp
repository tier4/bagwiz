// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/point_time.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <utility>
#include <vector>

namespace
{

using bagwiz::core::pointcloud::find_point_time_field;
using bagwiz::core::pointcloud::point_time_seconds;
using bagwiz::core::pointcloud::PointCloud2;
using bagwiz::core::pointcloud::PointField;
using bagwiz::core::pointcloud::PointFieldType;

PointCloud2 cloud_with(std::vector<PointField> fields)
{
  PointCloud2 c;
  c.fields = std::move(fields);
  return c;
}

}  // namespace

TEST(PointTime, FindsRecognisedNameAndType)
{
  const auto c =
    cloud_with({{"x", 0, PointFieldType::kFloat32, 1}, {"time", 12, PointFieldType::kUint32, 1}});
  const auto tf = find_point_time_field(c);
  ASSERT_TRUE(tf.has_value());
  EXPECT_EQ(tf->offset, 12u);
  EXPECT_EQ(tf->datatype, PointFieldType::kUint32);
}

TEST(PointTime, NamePrecedenceTBeatsTime)
{
  // Both "time" and "t" qualify; name precedence picks "t".
  const auto c =
    cloud_with({{"time", 4, PointFieldType::kFloat32, 1}, {"t", 8, PointFieldType::kFloat64, 1}});
  const auto tf = find_point_time_field(c);
  ASSERT_TRUE(tf.has_value());
  EXPECT_EQ(tf->offset, 8u);  // "t" wins over "time"
  EXPECT_EQ(tf->datatype, PointFieldType::kFloat64);
}

TEST(PointTime, RejectsCountNotOne)
{
  const auto c = cloud_with({{"time", 0, PointFieldType::kUint32, 2}});
  EXPECT_FALSE(find_point_time_field(c).has_value());
}

TEST(PointTime, RejectsUnsupportedDatatypeAndFallsThrough)
{
  // "t" is INT32 (unsupported) -> skipped; "time" is UINT32 -> used.
  const auto c =
    cloud_with({{"t", 0, PointFieldType::kInt32, 1}, {"time", 4, PointFieldType::kUint32, 1}});
  const auto tf = find_point_time_field(c);
  ASSERT_TRUE(tf.has_value());
  EXPECT_EQ(tf->offset, 4u);
  EXPECT_EQ(tf->datatype, PointFieldType::kUint32);
}

TEST(PointTime, NoTimeFieldIsNullopt)
{
  const auto c =
    cloud_with({{"x", 0, PointFieldType::kFloat32, 1}, {"ring", 4, PointFieldType::kUint16, 1}});
  EXPECT_FALSE(find_point_time_field(c).has_value());
}

TEST(PointTime, SecondsConversion)
{
  std::array<std::byte, 8> buf{};

  const std::uint32_t ns = 20'000'000;  // 20 ms
  std::memcpy(buf.data(), &ns, sizeof(ns));
  EXPECT_NEAR(point_time_seconds(buf.data(), {0, PointFieldType::kUint32}), 0.02, 1e-12);

  const float fs = 0.03F;
  std::memcpy(buf.data(), &fs, sizeof(fs));
  EXPECT_NEAR(point_time_seconds(buf.data(), {0, PointFieldType::kFloat32}), 0.03, 1e-6);

  const double ds = 1700.5;
  std::memcpy(buf.data(), &ds, sizeof(ds));
  EXPECT_NEAR(point_time_seconds(buf.data(), {0, PointFieldType::kFloat64}), 1700.5, 1e-12);
}
