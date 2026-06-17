// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/pointcloud2.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <string>
#include <vector>

namespace
{

class CdrBuilder
{
public:
  CdrBuilder()
  {
    for (int b : {0x00, 0x01, 0x00, 0x00}) {
      buf_.push_back(static_cast<std::byte>(b));
    }
  }
  void u8(std::uint8_t v) { buf_.push_back(static_cast<std::byte>(v)); }
  void u32(std::uint32_t v)
  {
    align(4);
    for (int i = 0; i < 4; ++i) {
      buf_.push_back(static_cast<std::byte>((v >> (8 * i)) & 0xFFU));
    }
  }
  void i32(std::int32_t v) { u32(static_cast<std::uint32_t>(v)); }
  void str(const std::string & s)
  {
    u32(static_cast<std::uint32_t>(s.size() + 1));
    for (char c : s) {
      buf_.push_back(static_cast<std::byte>(static_cast<unsigned char>(c)));
    }
    buf_.push_back(std::byte{0});
  }
  void byte_seq(std::span<const std::byte> b)
  {
    u32(static_cast<std::uint32_t>(b.size()));
    for (auto x : b) {
      buf_.push_back(x);
    }
  }
  [[nodiscard]] std::vector<std::byte> take() const { return buf_; }

private:
  void align(std::size_t n)
  {
    while ((buf_.size() - 4) % n != 0) {
      buf_.push_back(std::byte{0});
    }
  }
  std::vector<std::byte> buf_;
};

std::vector<std::byte> make_pointcloud2_payload()
{
  // Two points: x,y,float32 each.
  std::vector<std::byte> data(24, std::byte{0});
  float p1[3] = {1.0f, 2.0f, 3.0f};
  float p2[3] = {4.0f, 5.0f, 6.0f};
  std::memcpy(data.data(), p1, sizeof(p1));
  std::memcpy(data.data() + 12, p2, sizeof(p2));

  CdrBuilder b;
  b.i32(0);        // sec
  b.u32(0);        // nanosec
  b.str("lidar");  // frame_id
  b.u32(1);        // height
  b.u32(2);        // width
  b.u32(3);        // fields length
  b.str("x");      // field[0]
  b.u32(0);
  b.u8(7);  // float32
  b.u32(1);
  b.str("y");
  b.u32(4);
  b.u8(7);
  b.u32(1);
  b.str("z");
  b.u32(8);
  b.u8(7);
  b.u32(1);
  b.u8(0);    // is_bigendian
  b.u32(12);  // point_step
  b.u32(24);  // row_step
  b.byte_seq({data.data(), data.size()});
  b.u8(1);  // is_dense
  return b.take();
}

}  // namespace

TEST(PointCloud2Parser, ParsesBasicCloud)
{
  using bagwiz::core::pointcloud::parse_pointcloud2;
  const auto payload = make_pointcloud2_payload();
  const auto result = parse_pointcloud2(payload);
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.cloud->timestamp_ns, 0);
  EXPECT_EQ(result.cloud->frame_id, "lidar");
  EXPECT_EQ(result.cloud->width, 2u);
  EXPECT_EQ(result.cloud->fields.size(), 3u);
  EXPECT_EQ(result.cloud->point_step, 12u);
  EXPECT_TRUE(result.cloud->is_dense);
  ASSERT_EQ(result.cloud->data.size(), 24u);

  const auto off_x = result.cloud->field_offset("x");
  ASSERT_TRUE(off_x.has_value());
  EXPECT_EQ(*off_x, 0u);

  const auto off_y = result.cloud->field_offset("y");
  ASSERT_TRUE(off_y.has_value());
  EXPECT_EQ(*off_y, 4u);

  const auto off_z = result.cloud->field_offset("z");
  ASSERT_TRUE(off_z.has_value());
  EXPECT_EQ(*off_z, 8u);

  std::array<float, 3> first_point{};
  std::memcpy(first_point.data(), result.cloud->data.data(), sizeof(first_point));
  EXPECT_FLOAT_EQ(first_point[0], 1.0f);
  EXPECT_FLOAT_EQ(first_point[1], 2.0f);
  EXPECT_FLOAT_EQ(first_point[2], 3.0f);
}

TEST(PointCloud2Parser, ParseRejectsTruncatedPayload)
{
  using bagwiz::core::pointcloud::parse_pointcloud2;
  auto payload = make_pointcloud2_payload();
  // Truncate inside the data section (drop the last 5 bytes: part of data + is_dense).
  payload.resize(payload.size() - 5);
  const auto result = parse_pointcloud2(payload);
  EXPECT_FALSE(result.ok());
  EXPECT_FALSE(result.error.empty());
}

TEST(PointCloud2FieldOffset, MissingFieldReturnsNullopt)
{
  using bagwiz::core::pointcloud::PointCloud2;
  PointCloud2 cloud;
  cloud.fields = {{"x", 0}, {"y", 4}};
  const auto offset = cloud.field_offset("z");
  EXPECT_FALSE(offset.has_value());
}
