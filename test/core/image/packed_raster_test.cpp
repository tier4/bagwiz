// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/image/packed_raster.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <span>
#include <string>
#include <vector>

namespace
{
using bagwiz::core::image::is_supported_image_type;
using bagwiz::core::image::to_packed_raster;

constexpr const char * kImageType = "sensor_msgs/msg/Image";
constexpr const char * kCompressedImageType = "sensor_msgs/msg/CompressedImage";

// Little-endian CDR-1 builder mirroring the wire layout the production CdrReader
// consumes (same helper used by raw_image_test.cpp / compressed_image_test.cpp).
class CdrBuilder
{
public:
  CdrBuilder()
  {
    for (int b : {0x00, 0x01, 0x00, 0x00}) {  // rep_id high=0, low=1 (LE), options=0
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
    u32(static_cast<std::uint32_t>(s.size() + 1));  // length includes the trailing NUL
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

std::vector<std::byte> to_vec(std::initializer_list<int> vals)
{
  std::vector<std::byte> v;
  v.reserve(vals.size());
  for (int x : vals) {
    v.push_back(static_cast<std::byte>(x & 0xFF));
  }
  return v;
}

std::vector<std::byte> make_image(
  std::uint32_t width, std::uint32_t height, std::uint32_t step, const std::string & encoding,
  std::span<const std::byte> data)
{
  CdrBuilder b;
  b.i32(0);      // header.stamp.sec
  b.u32(0);      // header.stamp.nanosec
  b.str("cam");  // header.frame_id
  b.u32(height);
  b.u32(width);
  b.str(encoding);
  b.u8(0);  // is_bigendian
  b.u32(step);
  b.byte_seq(data);
  return b.take();
}

std::vector<std::byte> make_compressed(const std::string & format, std::span<const std::byte> data)
{
  CdrBuilder b;
  b.i32(0);      // header.stamp.sec
  b.u32(0);      // header.stamp.nanosec
  b.str("cam");  // header.frame_id
  b.str(format);
  b.byte_seq(data);
  return b.take();
}

// --- raw sensor_msgs/msg/Image ------------------------------------------------

TEST(ToPackedRasterTest, Bgr8TightlyPackedIsCopiedVerbatim)
{
  const auto data = to_vec({0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11});  // 2x2, 3ch
  const auto payload = make_image(2, 2, 6, "bgr8", {data.data(), data.size()});

  const auto r = to_packed_raster(kImageType, {payload.data(), payload.size()});
  ASSERT_TRUE(r.ok()) << r.error;
  EXPECT_EQ(r.raster->width, 2U);
  EXPECT_EQ(r.raster->height, 2U);
  EXPECT_EQ(r.raster->encoding, "bgr8");
  ASSERT_EQ(r.raster->bgr.size(), data.size());
  EXPECT_EQ(r.raster->bgr, data);
}

TEST(ToPackedRasterTest, Rgb8IsSwappedToBgr)
{
  const auto data = to_vec({1, 2, 3, 4, 5, 6});  // 2x1, rgb pixels (1,2,3) (4,5,6)
  const auto payload = make_image(2, 1, 6, "rgb8", {data.data(), data.size()});

  const auto r = to_packed_raster(kImageType, {payload.data(), payload.size()});
  ASSERT_TRUE(r.ok()) << r.error;
  EXPECT_EQ(r.raster->encoding, "rgb8");  // source encoding preserved for the caption
  // R/B swapped per pixel: (1,2,3)->(3,2,1), (4,5,6)->(6,5,4)
  EXPECT_EQ(r.raster->bgr, to_vec({3, 2, 1, 6, 5, 4}));
}

TEST(ToPackedRasterTest, RowStridePaddingIsStripped)
{
  // 2x2 bgr8 with step=8 (2 padding bytes per 6-byte row).
  const auto data = to_vec({0, 1, 2, 3, 4, 5, 90, 91, 8, 9, 10, 11, 12, 13, 92, 93});
  const auto payload = make_image(2, 2, 8, "bgr8", {data.data(), data.size()});

  const auto r = to_packed_raster(kImageType, {payload.data(), payload.size()});
  ASSERT_TRUE(r.ok()) << r.error;
  ASSERT_EQ(r.raster->bgr.size(), 2U * 2U * 3U);  // padding removed
  EXPECT_EQ(r.raster->bgr, to_vec({0, 1, 2, 3, 4, 5, 8, 9, 10, 11, 12, 13}));
}

TEST(ToPackedRasterTest, UnsupportedRawEncodingIsError)
{
  const auto data = to_vec({0, 0, 0});
  const auto payload = make_image(1, 1, 3, "mono8", {data.data(), data.size()});

  const auto r = to_packed_raster(kImageType, {payload.data(), payload.size()});
  EXPECT_FALSE(r.ok());
  EXPECT_FALSE(r.raster.has_value());
  EXPECT_NE(r.error.find("mono8"), std::string::npos) << r.error;
}

TEST(ToPackedRasterTest, TruncatedRawPayloadIsError)
{
  const auto data = to_vec({0, 1, 2, 3, 4, 5});
  auto payload = make_image(2, 1, 6, "bgr8", {data.data(), data.size()});
  payload.resize(payload.size() - 3);  // chop into the pixel sequence

  const auto r = to_packed_raster(kImageType, {payload.data(), payload.size()});
  EXPECT_FALSE(r.ok());
  EXPECT_FALSE(r.error.empty());
}

TEST(ToPackedRasterTest, ZeroDimensionRawImageIsError)
{
  const auto payload = make_image(0, 0, 0, "bgr8", {});

  const auto r = to_packed_raster(kImageType, {payload.data(), payload.size()});
  EXPECT_FALSE(r.ok());
  EXPECT_FALSE(r.error.empty());
}

// --- type dispatch & compressed ----------------------------------------------

TEST(ToPackedRasterTest, UnsupportedMessageTypeIsError)
{
  const auto data = to_vec({0, 1, 2});
  const auto payload = make_image(1, 1, 3, "bgr8", {data.data(), data.size()});

  const auto r = to_packed_raster("std_msgs/msg/String", {payload.data(), payload.size()});
  EXPECT_FALSE(r.ok());
  EXPECT_NE(r.error.find("unsupported image message type"), std::string::npos) << r.error;
}

TEST(ToPackedRasterTest, CompressedGarbageBitstreamIsError)
{
  // Routes through the libav-backed decoder, which reports an error (never
  // throws) on a bitstream with no recognizable JPEG/PNG magic.
  const auto blob = to_vec({0x01, 0x02, 0x03, 0x04, 0x05});
  const auto payload = make_compressed("png", {blob.data(), blob.size()});

  const auto r = to_packed_raster(kCompressedImageType, {payload.data(), payload.size()});
  EXPECT_FALSE(r.ok());
  EXPECT_FALSE(r.error.empty());
}

TEST(ToPackedRasterTest, IsSupportedImageTypeMatchesDispatch)
{
  EXPECT_TRUE(is_supported_image_type(kImageType));
  EXPECT_TRUE(is_supported_image_type(kCompressedImageType));
  EXPECT_FALSE(is_supported_image_type("std_msgs/msg/String"));
  EXPECT_FALSE(is_supported_image_type("sensor_msgs/msg/PointCloud2"));
  EXPECT_FALSE(is_supported_image_type(""));
}

}  // namespace
