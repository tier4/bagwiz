// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/lidar_scan.hpp"

#include "bagwiz/core/pointcloud/pointcloud2.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace
{

namespace pc = bagwiz::core::pointcloud;
namespace slam = bagwiz::core::slam;

// Append the raw little-endian bytes of `value` to `out`.
template <typename T>
void put(std::vector<std::byte> & out, T value)
{
  std::array<std::byte, sizeof(T)> bytes{};
  std::memcpy(bytes.data(), &value, sizeof(T));
  out.insert(out.end(), bytes.begin(), bytes.end());
}

pc::PointField field(const std::string & name, std::uint32_t offset, pc::PointFieldType dt)
{
  pc::PointField f;
  f.name = name;
  f.offset = offset;
  f.datatype = dt;
  f.count = 1;
  return f;
}

// A cloud with x,y,z float32 at offsets 0/4/8, point_step 12, no intensity/time.
pc::PointCloud2 make_xyz_cloud()
{
  pc::PointCloud2 c;
  c.timestamp_ns = 1'500'000'000'000'000'000LL;
  c.frame_id = "base_link";
  c.height = 1;
  c.width = 2;
  c.point_step = 12;
  c.row_step = 24;
  c.is_bigendian = false;
  c.is_dense = true;
  c.fields = {
    field("x", 0, pc::PointFieldType::kFloat32),
    field("y", 4, pc::PointFieldType::kFloat32),
    field("z", 8, pc::PointFieldType::kFloat32),
  };
  // point 0 = (1,2,3), point 1 = (4,5,6)
  put<float>(c.data, 1.0F);
  put<float>(c.data, 2.0F);
  put<float>(c.data, 3.0F);
  put<float>(c.data, 4.0F);
  put<float>(c.data, 5.0F);
  put<float>(c.data, 6.0F);
  return c;
}

TEST(LidarScan, ExtractsXyzOnly)
{
  const auto r = slam::to_lidar_scan(make_xyz_cloud());
  ASSERT_TRUE(r.ok()) << r.error;
  const auto & s = r.scan.value();
  EXPECT_EQ(s.frame_id, "base_link");
  EXPECT_EQ(s.stamp_ns, 1'500'000'000'000'000'000LL);
  ASSERT_EQ(s.points.size(), 2U);
  EXPECT_DOUBLE_EQ(s.points[0][0], 1.0);
  EXPECT_DOUBLE_EQ(s.points[0][1], 2.0);
  EXPECT_DOUBLE_EQ(s.points[0][2], 3.0);
  EXPECT_DOUBLE_EQ(s.points[1][0], 4.0);
  EXPECT_DOUBLE_EQ(s.points[1][2], 6.0);
  EXPECT_TRUE(s.intensities.empty());
  EXPECT_TRUE(s.times.empty());
  EXPECT_FALSE(s.has_per_point_time);
}

TEST(LidarScan, ExtractsIntensityAndRelativeFloatTime)
{
  pc::PointCloud2 c;
  c.frame_id = "lidar";
  c.height = 1;
  c.width = 2;
  c.point_step = 20;  // x,y,z f32 (12) + intensity u8 (1) + pad(3) + t f32 @16
  c.row_step = 40;
  c.fields = {
    field("x", 0, pc::PointFieldType::kFloat32),
    field("y", 4, pc::PointFieldType::kFloat32),
    field("z", 8, pc::PointFieldType::kFloat32),
    field("intensity", 12, pc::PointFieldType::kUint8),
    field("t", 16, pc::PointFieldType::kFloat32),
  };
  for (int i = 0; i < 2; ++i) {
    put<float>(c.data, static_cast<float>(i));                      // x
    put<float>(c.data, 0.0F);                                       // y
    put<float>(c.data, 0.0F);                                       // z
    put<std::uint8_t>(c.data, static_cast<std::uint8_t>(100 + i));  // intensity
    put<std::uint8_t>(c.data, std::uint8_t{0});                     // pad
    put<std::uint8_t>(c.data, std::uint8_t{0});                     // pad
    put<std::uint8_t>(c.data, std::uint8_t{0});                     // pad
    put<float>(c.data, 0.05F * static_cast<float>(i));              // t (relative seconds)
  }
  const auto r = slam::to_lidar_scan(c);
  ASSERT_TRUE(r.ok()) << r.error;
  const auto & s = r.scan.value();
  ASSERT_EQ(s.intensities.size(), 2U);
  EXPECT_DOUBLE_EQ(s.intensities[0], 100.0);
  EXPECT_DOUBLE_EQ(s.intensities[1], 101.0);
  ASSERT_EQ(s.times.size(), 2U);
  EXPECT_TRUE(s.has_per_point_time);
  EXPECT_NEAR(s.times[0], 0.0, 1e-9);
  EXPECT_NEAR(s.times[1], 0.05, 1e-6);
}

TEST(LidarScan, Uint32TimeIsScaledToSeconds)
{
  pc::PointCloud2 c;
  c.height = 1;
  c.width = 1;
  c.point_step = 16;  // xyz f32 + t u32 @12
  c.fields = {
    field("x", 0, pc::PointFieldType::kFloat32),
    field("y", 4, pc::PointFieldType::kFloat32),
    field("z", 8, pc::PointFieldType::kFloat32),
    field("timestamp", 12, pc::PointFieldType::kUint32),
  };
  put<float>(c.data, 0.0F);
  put<float>(c.data, 0.0F);
  put<float>(c.data, 0.0F);
  put<std::uint32_t>(c.data, 500'000'000U);  // 0.5 s in ns
  const auto r = slam::to_lidar_scan(c);
  ASSERT_TRUE(r.ok()) << r.error;
  ASSERT_EQ(r.scan->times.size(), 1U);
  EXPECT_TRUE(r.scan->has_per_point_time);
  EXPECT_NEAR(r.scan->times[0], 0.5, 1e-9);
}

TEST(LidarScan, MissingCoordinateFieldIsError)
{
  pc::PointCloud2 c = make_xyz_cloud();
  c.fields.pop_back();  // drop z
  const auto r = slam::to_lidar_scan(c);
  EXPECT_FALSE(r.ok());
  EXPECT_FALSE(r.error.empty());
}

TEST(LidarScan, NonFloatCoordinateIsError)
{
  pc::PointCloud2 c = make_xyz_cloud();
  c.fields[0].datatype = pc::PointFieldType::kInt32;  // x not float
  const auto r = slam::to_lidar_scan(c);
  EXPECT_FALSE(r.ok());
}

TEST(LidarScan, BigEndianIsError)
{
  pc::PointCloud2 c = make_xyz_cloud();
  c.is_bigendian = true;
  const auto r = slam::to_lidar_scan(c);
  EXPECT_FALSE(r.ok());
}

}  // namespace
