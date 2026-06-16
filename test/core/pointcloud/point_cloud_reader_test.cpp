// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/pointcloud/point_cloud_reader.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <span>
#include <string>
#include <tuple>
#include <vector>

namespace
{
using bagwiz::core::pointcloud::extract_point_cloud;
using bagwiz::core::pointcloud::is_valid_point;
using bagwiz::core::pointcloud::read_float_field;
using bagwiz::core::pointcloud::read_intensity;

struct FieldDesc
{
  std::string name;
  std::uint32_t offset;
  std::uint8_t datatype;
  std::uint32_t count = 1;
};

std::array<std::byte, 4> f32_bytes(float v, bool big_endian = false)
{
  std::array<std::byte, 4> bytes{};
  std::memcpy(bytes.data(), &v, sizeof(v));
  if (big_endian != (std::endian::native == std::endian::big)) {
    std::reverse(bytes.begin(), bytes.end());
  }
  return bytes;
}

std::array<std::byte, 2> u16_bytes(std::uint16_t v, bool big_endian = false)
{
  std::array<std::byte, 2> bytes{
    static_cast<std::byte>(v & 0xFFU),
    static_cast<std::byte>((v >> 8) & 0xFFU)};
  if (big_endian) {
    std::reverse(bytes.begin(), bytes.end());
  }
  return bytes;
}

void append_bytes(std::vector<std::byte> & dst, std::span<const std::byte> src)
{
  for (auto b : src) {
    dst.push_back(b);
  }
}

// CDR-1 buffer builder that can emit either little-endian (kind=1) or
// big-endian (kind=0) payloads. Multi-byte primitives are byte-swapped so the
// wire representation matches the selected endianness.
class CdrBuilder
{
public:
  explicit CdrBuilder(bool big_endian = false) : big_endian_(big_endian)
  {
    buf_.push_back(std::byte{0});
    buf_.push_back(big_endian ? std::byte{0} : std::byte{1});
    buf_.push_back(std::byte{0});
    buf_.push_back(std::byte{0});
  }

  void u8(std::uint8_t v) { buf_.push_back(static_cast<std::byte>(v)); }

  void u16(std::uint16_t v)
  {
    align(2);
    if (big_endian_) {
      for (int i = 1; i >= 0; --i) {
        buf_.push_back(static_cast<std::byte>((v >> (8 * i)) & 0xFFU));
      }
    } else {
      for (int i = 0; i < 2; ++i) {
        buf_.push_back(static_cast<std::byte>((v >> (8 * i)) & 0xFFU));
      }
    }
  }

  void u32(std::uint32_t v)
  {
    align(4);
    if (big_endian_) {
      for (int i = 3; i >= 0; --i) {
        buf_.push_back(static_cast<std::byte>((v >> (8 * i)) & 0xFFU));
      }
    } else {
      for (int i = 0; i < 4; ++i) {
        buf_.push_back(static_cast<std::byte>((v >> (8 * i)) & 0xFFU));
      }
    }
  }

  void i32(std::int32_t v) { u32(static_cast<std::uint32_t>(v)); }

  void bool_(bool v) { u8(v ? 1U : 0U); }

  void str(const std::string & s)
  {
    u32(static_cast<std::uint32_t>(s.size() + 1));
    for (char c : s) {
      buf_.push_back(static_cast<std::byte>(static_cast<unsigned char>(c)));
    }
    buf_.push_back(std::byte{0});
  }

  void f32(float v)
  {
    align(4);
    for (auto b : f32_bytes(v, big_endian_)) {
      buf_.push_back(b);
    }
  }

  void field(
    const std::string & name, std::uint32_t offset, std::uint8_t datatype, std::uint32_t count = 1)
  {
    str(name);
    u32(offset);
    u8(datatype);
    u32(count);
  }

  void byte_seq(std::span<const std::byte> b)
  {
    u32(static_cast<std::uint32_t>(b.size()));
    for (auto x : b) {
      buf_.push_back(x);
    }
  }

  [[nodiscard]] std::span<const std::byte> view() const { return {buf_.data(), buf_.size()}; }

private:
  void align(std::size_t n)
  {
    while ((buf_.size() - 4) % n != 0) {
      buf_.push_back(std::byte{0});
    }
  }

  bool big_endian_ = false;
  std::vector<std::byte> buf_;
};

std::vector<std::byte> make_payload(
  const std::vector<FieldDesc> & fields,
  std::uint32_t width,
  std::uint32_t height,
  std::uint32_t point_step,
  std::span<const std::byte> data,
  bool big_endian = false,
  std::uint32_t row_step = 0)
{
  CdrBuilder b(big_endian);
  b.i32(0);                                          // header.stamp.sec
  b.u32(0);                                          // header.stamp.nanosec
  b.str("lidar");                                    // header.frame_id
  b.u32(height);                                     // height
  b.u32(width);                                      // width
  b.u32(static_cast<std::uint32_t>(fields.size()));  // fields[] length
  for (const auto & f : fields) {
    b.field(f.name, f.offset, f.datatype, f.count);
  }
  b.bool_(big_endian);                               // is_bigendian
  b.u32(point_step);                                 // point_step
  b.u32(row_step == 0 ? point_step * width : row_step);  // row_step
  b.byte_seq(data);
  b.bool_(true);                                     // is_dense
  return std::vector<std::byte>(b.view().begin(), b.view().end());
}

std::vector<std::byte> make_xyz_uint8_points(
  const std::vector<std::tuple<float, float, float, std::uint8_t>> & points, bool big_endian = false)
{
  std::vector<std::byte> data;
  for (const auto & p : points) {
    append_bytes(data, f32_bytes(std::get<0>(p), big_endian));
    append_bytes(data, f32_bytes(std::get<1>(p), big_endian));
    append_bytes(data, f32_bytes(std::get<2>(p), big_endian));
    data.push_back(static_cast<std::byte>(std::get<3>(p)));
    // point_step = 16, pad to keep points aligned.
    data.push_back(std::byte{0});
    data.push_back(std::byte{0});
    data.push_back(std::byte{0});
  }
  return data;
}

TEST(PointCloudReaderTest, ExtractsXYZIntensityUint8)
{
  const auto data = make_xyz_uint8_points({
    {1.0f, 2.0f, 3.0f, 100},
    {4.0f, 5.0f, 6.0f, 200},
  });
  const auto payload = make_payload(
    {
      {"x", 0, 7},
      {"y", 4, 7},
      {"z", 8, 7},
      {"intensity", 12, 1},
    },
    2, 1, 16, data);

  const auto result = extract_point_cloud(payload);
  ASSERT_TRUE(result.ok()) << result.error;
  EXPECT_EQ(result.view->width, 2U);
  EXPECT_EQ(result.view->height, 1U);
  EXPECT_EQ(result.view->frame_id, "lidar");
  EXPECT_EQ(result.view->point_step, 16U);
  ASSERT_TRUE(result.view->x_offset.has_value());
  ASSERT_TRUE(result.view->y_offset.has_value());
  ASSERT_TRUE(result.view->z_offset.has_value());
  ASSERT_TRUE(result.view->intensity_offset.has_value());
  EXPECT_EQ(*result.view->x_offset, 0U);
  EXPECT_EQ(*result.view->y_offset, 4U);
  EXPECT_EQ(*result.view->z_offset, 8U);
  EXPECT_EQ(*result.view->intensity_offset, 12U);
  EXPECT_EQ(result.view->intensity_datatype, 1U);

  const auto x0 = read_float_field(*result.view, 0, *result.view->x_offset);
  ASSERT_TRUE(x0.has_value());
  EXPECT_FLOAT_EQ(*x0, 1.0f);
  const auto y0 = read_float_field(*result.view, 0, *result.view->y_offset);
  ASSERT_TRUE(y0.has_value());
  EXPECT_FLOAT_EQ(*y0, 2.0f);
  const auto z1 = read_float_field(*result.view, 1, *result.view->z_offset);
  ASSERT_TRUE(z1.has_value());
  EXPECT_FLOAT_EQ(*z1, 6.0f);

  const auto i0 = read_intensity(*result.view, 0);
  ASSERT_TRUE(i0.has_value());
  EXPECT_FLOAT_EQ(*i0, 100.0f / 255.0f);
  const auto i1 = read_intensity(*result.view, 1);
  ASSERT_TRUE(i1.has_value());
  EXPECT_FLOAT_EQ(*i1, 200.0f / 255.0f);
}

TEST(PointCloudReaderTest, ExtractsXYZIntensityUint16)
{
  // 2 points: x/y/z float32 (12 bytes) + uint16 intensity + 2 pad = 16 bytes.
  std::vector<std::byte> data;
  const auto append_point = [&](float x, float y, float z, std::uint16_t intensity) {
    append_bytes(data, f32_bytes(x));
    append_bytes(data, f32_bytes(y));
    append_bytes(data, f32_bytes(z));
    append_bytes(data, u16_bytes(intensity));
    data.push_back(std::byte{0});
    data.push_back(std::byte{0});
  };
  append_point(1.0f, 2.0f, 3.0f, 0);
  append_point(4.0f, 5.0f, 6.0f, 65535);

  const auto payload = make_payload(
    {
      {"x", 0, 7},
      {"y", 4, 7},
      {"z", 8, 7},
      {"intensity", 12, 2},
    },
    2, 1, 16, data);

  const auto result = extract_point_cloud(payload);
  ASSERT_TRUE(result.ok()) << result.error;
  ASSERT_TRUE(result.view->intensity_offset.has_value());
  EXPECT_EQ(result.view->intensity_datatype, 2U);
  const auto i0 = read_intensity(*result.view, 0);
  ASSERT_TRUE(i0.has_value());
  EXPECT_FLOAT_EQ(*i0, 0.0f);
  const auto i1 = read_intensity(*result.view, 1);
  ASSERT_TRUE(i1.has_value());
  EXPECT_FLOAT_EQ(*i1, 1.0f);
}

TEST(PointCloudReaderTest, ExtractsXYZIntensityFloat32)
{
  // 2 points: x/y/z float32 + float32 intensity = 16 bytes.
  std::vector<std::byte> data;
  const auto append_point = [&](float x, float y, float z, float intensity) {
    for (float v : {x, y, z, intensity}) {
      append_bytes(data, f32_bytes(v));
    }
  };
  append_point(1.0f, 2.0f, 3.0f, 0.25f);
  append_point(4.0f, 5.0f, 6.0f, 0.75f);

  const auto payload = make_payload(
    {
      {"x", 0, 7},
      {"y", 4, 7},
      {"z", 8, 7},
      {"intensity", 12, 7},
    },
    2, 1, 16, data);

  const auto result = extract_point_cloud(payload);
  ASSERT_TRUE(result.ok()) << result.error;
  ASSERT_TRUE(result.view->intensity_offset.has_value());
  EXPECT_EQ(result.view->intensity_datatype, 7U);
  const auto i0 = read_intensity(*result.view, 0);
  ASSERT_TRUE(i0.has_value());
  EXPECT_FLOAT_EQ(*i0, 0.25f);
  const auto i1 = read_intensity(*result.view, 1);
  ASSERT_TRUE(i1.has_value());
  EXPECT_FLOAT_EQ(*i1, 0.75f);
}

TEST(PointCloudReaderTest, ExtractsWithoutIntensity)
{
  std::vector<std::byte> data;
  for (float v : {1.0f, 2.0f, 3.0f}) {
    append_bytes(data, f32_bytes(v));
  }
  const auto payload = make_payload(
    {
      {"x", 0, 7},
      {"y", 4, 7},
      {"z", 8, 7},
    },
    1, 1, 12, data);

  const auto result = extract_point_cloud(payload);
  ASSERT_TRUE(result.ok()) << result.error;
  EXPECT_FALSE(result.view->intensity_offset.has_value());
  EXPECT_EQ(result.view->intensity_datatype, 0U);
  EXPECT_EQ(read_intensity(*result.view, 0), std::nullopt);
}

TEST(PointCloudReaderTest, ExtractsXYZBigEndian)
{
  constexpr bool big_endian = true;
  std::vector<std::byte> data;
  for (float v : {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f}) {
    append_bytes(data, f32_bytes(v, big_endian));
  }
  const auto payload = make_payload(
    {
      {"x", 0, 7},
      {"y", 4, 7},
      {"z", 8, 7},
    },
    2, 1, 12, data, big_endian);

  const auto result = extract_point_cloud(payload);
  ASSERT_TRUE(result.ok()) << result.error;
  EXPECT_TRUE(result.view->is_bigendian);
  const auto x0 = read_float_field(*result.view, 0, *result.view->x_offset);
  ASSERT_TRUE(x0.has_value());
  EXPECT_FLOAT_EQ(*x0, 1.0f);
  const auto y1 = read_float_field(*result.view, 1, *result.view->y_offset);
  ASSERT_TRUE(y1.has_value());
  EXPECT_FLOAT_EQ(*y1, 5.0f);
  const auto z1 = read_float_field(*result.view, 1, *result.view->z_offset);
  ASSERT_TRUE(z1.has_value());
  EXPECT_FLOAT_EQ(*z1, 6.0f);
}

TEST(PointCloudReaderTest, RejectsMissingX)
{
  std::vector<std::byte> data;
  for (float v : {2.0f, 3.0f}) {
    append_bytes(data, f32_bytes(v));
  }
  const auto payload = make_payload(
    {
      {"y", 0, 7},
      {"z", 4, 7},
    },
    1, 1, 8, data);

  const auto result = extract_point_cloud(payload);
  EXPECT_FALSE(result.ok());
  EXPECT_FALSE(result.error.empty());
}

TEST(PointCloudReaderTest, RejectsMissingZ)
{
  std::vector<std::byte> data;
  for (float v : {1.0f, 2.0f}) {
    append_bytes(data, f32_bytes(v));
  }
  const auto payload = make_payload(
    {
      {"x", 0, 7},
      {"y", 4, 7},
    },
    1, 1, 8, data);

  const auto result = extract_point_cloud(payload);
  EXPECT_FALSE(result.ok());
  EXPECT_FALSE(result.error.empty());
}

TEST(PointCloudReaderTest, RejectsWrongXDatatype)
{
  std::vector<std::byte> data;
  for (float v : {1.0f, 2.0f, 3.0f}) {
    append_bytes(data, f32_bytes(v));
  }
  const auto payload = make_payload(
    {
      {"x", 0, 1},  // UINT8 in the task's convention, not FLOAT32.
      {"y", 4, 7},
      {"z", 8, 7},
    },
    1, 1, 12, data);

  const auto result = extract_point_cloud(payload);
  EXPECT_FALSE(result.ok());
  EXPECT_FALSE(result.error.empty());
}

TEST(PointCloudReaderTest, RejectsXCountNotOne)
{
  std::vector<std::byte> data;
  for (float v : {1.0f, 2.0f, 3.0f}) {
    append_bytes(data, f32_bytes(v));
  }
  const auto payload = make_payload(
    {
      {"x", 0, 7, 2},
      {"y", 4, 7},
      {"z", 8, 7},
    },
    1, 1, 12, data);

  const auto result = extract_point_cloud(payload);
  EXPECT_FALSE(result.ok());
  EXPECT_FALSE(result.error.empty());
}

TEST(PointCloudReaderTest, RejectsUnsupportedIntensityDatatype)
{
  std::vector<std::byte> data;
  for (float v : {1.0f, 2.0f, 3.0f}) {
    append_bytes(data, f32_bytes(v));
  }
  // INT16 (datatype 3) is not a supported intensity type.
  const auto payload = make_payload(
    {
      {"x", 0, 7},
      {"y", 4, 7},
      {"z", 8, 7},
      {"intensity", 12, 3},
    },
    1, 1, 12, data);

  const auto result = extract_point_cloud(payload);
  EXPECT_FALSE(result.ok());
  EXPECT_FALSE(result.error.empty());
}

TEST(PointCloudReaderTest, RejectsDataSizeMismatch)
{
  std::vector<std::byte> data;
  for (float v : {1.0f, 2.0f, 3.0f}) {
    append_bytes(data, f32_bytes(v));
  }
  // Claim row_step is larger than the actual data provided.
  const auto payload = make_payload(
    {
      {"x", 0, 7},
      {"y", 4, 7},
      {"z", 8, 7},
    },
    1, 1, 12, data, false, 16);

  const auto result = extract_point_cloud(payload);
  EXPECT_FALSE(result.ok());
  EXPECT_FALSE(result.error.empty());
}

TEST(PointCloudReaderTest, SkipsNanOrInfinitePoints)
{
  std::vector<std::byte> data;
  const auto append_point = [&](float x, float y, float z) {
    for (float v : {x, y, z}) {
      append_bytes(data, f32_bytes(v));
    }
  };
  append_point(1.0f, 2.0f, 3.0f);
  append_point(std::numeric_limits<float>::quiet_NaN(), 2.0f, 3.0f);
  append_point(1.0f, std::numeric_limits<float>::infinity(), 3.0f);
  append_point(1.0f, 2.0f, -std::numeric_limits<float>::infinity());

  const auto payload = make_payload(
    {
      {"x", 0, 7},
      {"y", 4, 7},
      {"z", 8, 7},
    },
    4, 1, 12, data);

  const auto result = extract_point_cloud(payload);
  ASSERT_TRUE(result.ok()) << result.error;
  EXPECT_TRUE(is_valid_point(*result.view, 0));
  EXPECT_FALSE(is_valid_point(*result.view, 1));
  EXPECT_FALSE(is_valid_point(*result.view, 2));
  EXPECT_FALSE(is_valid_point(*result.view, 3));
}

TEST(PointCloudReaderTest, ReadFloatFieldRejectsOutOfBounds)
{
  std::vector<std::byte> data;
  for (float v : {1.0f, 2.0f, 3.0f}) {
    append_bytes(data, f32_bytes(v));
  }
  const auto payload = make_payload(
    {
      {"x", 0, 7},
      {"y", 4, 7},
      {"z", 8, 7},
    },
    1, 1, 12, data);

  const auto result = extract_point_cloud(payload);
  ASSERT_TRUE(result.ok()) << result.error;
  const auto x0 = read_float_field(*result.view, 0, *result.view->x_offset);
  ASSERT_TRUE(x0.has_value());
  EXPECT_FLOAT_EQ(*x0, 1.0f);
  EXPECT_EQ(read_float_field(*result.view, 1, *result.view->x_offset), std::nullopt);
  EXPECT_EQ(read_intensity(*result.view, 1), std::nullopt);
}

TEST(PointCloudReaderTest, TruncatedPayloadYieldsError)
{
  CdrBuilder b;
  b.i32(0);
  b.u32(0);
  b.str("lidar");
  b.u32(1);
  b.u32(1);
  b.u32(0);  // no fields, will fail later but the payload is already truncated.
  const auto result = extract_point_cloud(b.view());
  EXPECT_FALSE(result.ok());
  EXPECT_FALSE(result.error.empty());
}

}  // namespace
