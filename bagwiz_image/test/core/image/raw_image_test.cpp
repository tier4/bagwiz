// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/image/raw_image.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace
{
using bagwiz::core::image::extract_raw_image;

// Minimal little-endian CDR-1 buffer builder mirroring the wire layout the
// production CdrReader consumes. Alignment is relative to the 4-byte
// encapsulation header (the "body offset"), matching CdrReader's
// (offset - 4) % size rule.
class CdrBuilder
{
public:
  CdrBuilder()
  {
    for (int b : {0x00, 0x01, 0x00, 0x00}) {  // rep_id high=0, low=1 (LE), options=0,0
      buf_.push_back(static_cast<std::byte>(b));
    }
  }

  void u8(std::uint8_t v) { buf_.push_back(static_cast<std::byte>(v)); }
  void u32(std::uint32_t v)
  {
    align(4);
    for (int i = 0; i < 4; ++i) {
      buf_.push_back(static_cast<std::byte>((v >> (8 * i)) & 0xFFU));  // little-endian
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
  // uint8[] sequence: uint32 length prefix + raw element bytes (no per-element
  // alignment for 1-byte elements).
  void byte_seq(std::span<const std::byte> b)
  {
    u32(static_cast<std::uint32_t>(b.size()));
    for (auto x : b) {
      buf_.push_back(x);
    }
  }
  // Raw aligned u32 with no following payload — used to forge malformed inputs.
  void raw_u32(std::uint32_t v) { u32(v); }

  [[nodiscard]] std::span<const std::byte> view() const { return {buf_.data(), buf_.size()}; }

private:
  void align(std::size_t n)
  {
    while ((buf_.size() - 4) % n != 0) {
      buf_.push_back(std::byte{0});
    }
  }
  std::vector<std::byte> buf_;
};

std::vector<std::byte> iota_bytes(std::size_t n)
{
  std::vector<std::byte> v(n);
  for (std::size_t i = 0; i < n; ++i) {
    v[i] = static_cast<std::byte>(i & 0xFFU);
  }
  return v;
}

// Build a valid sensor_msgs/msg/Image CDR payload. `stamp_ns` sets header.stamp
// (sec/nanosec split); the default of 0 leaves it unset.
CdrBuilder make_image(
  std::uint32_t width, std::uint32_t height, std::uint32_t step, const std::string & encoding,
  std::span<const std::byte> data, std::int64_t stamp_ns = 0)
{
  CdrBuilder b;
  b.i32(static_cast<std::int32_t>(stamp_ns / 1'000'000'000LL));   // header.stamp.sec
  b.u32(static_cast<std::uint32_t>(stamp_ns % 1'000'000'000LL));  // header.stamp.nanosec
  b.str("cam");                                                   // header.frame_id
  b.u32(height);
  b.u32(width);
  b.str(encoding);
  b.u8(0);  // is_bigendian
  b.u32(step);
  b.byte_seq(data);
  return b;
}

TEST(RawImageTest, ParsesTightlyPackedBgr8)
{
  const auto data = iota_bytes(2 * 2 * 3);  // 2x2, 3 channels, no padding
  const auto buf = make_image(2, 2, 6, "bgr8", data);
  const auto r = extract_raw_image(buf.view());
  ASSERT_TRUE(r.ok()) << r.error;
  EXPECT_EQ(r.image->width, 2U);
  EXPECT_EQ(r.image->height, 2U);
  EXPECT_EQ(r.image->step, 6U);
  EXPECT_EQ(r.image->encoding, "bgr8");
  ASSERT_EQ(r.image->data.size(), data.size());
  // Zero-copy: the view's data points into the original payload buffer (the
  // pixel bytes are the tail of the serialized message).
  EXPECT_EQ(r.image->data.data(), buf.view().data() + (buf.view().size() - data.size()));
}

TEST(RawImageTest, ParsesHeaderStamp)
{
  const auto data = iota_bytes(2 * 2 * 3);
  // 1700000000.250000000 s -> 1700000000250000000 ns
  const auto buf = make_image(2, 2, 6, "bgr8", data, 1'700'000'000'250'000'000LL);
  const auto r = extract_raw_image(buf.view());
  ASSERT_TRUE(r.ok()) << r.error;
  EXPECT_EQ(r.image->header_stamp_ns, 1'700'000'000'250'000'000LL);
}

TEST(RawImageTest, UnsetHeaderStampIsZero)
{
  const auto data = iota_bytes(2 * 2 * 3);
  const auto buf = make_image(2, 2, 6, "bgr8", data);  // default stamp 0
  const auto r = extract_raw_image(buf.view());
  ASSERT_TRUE(r.ok()) << r.error;
  EXPECT_EQ(r.image->header_stamp_ns, 0);
}

TEST(RawImageTest, ParsesRowPaddedImage)
{
  const auto data = iota_bytes(2 * 8);  // step (8) > width*channels (6): 2 pad bytes/row
  const auto buf = make_image(2, 2, 8, "rgb8", data);
  const auto r = extract_raw_image(buf.view());
  ASSERT_TRUE(r.ok()) << r.error;
  EXPECT_EQ(r.image->step, 8U);
  EXPECT_EQ(r.image->encoding, "rgb8");
  EXPECT_EQ(r.image->data.size(), 16U);
}

TEST(RawImageTest, TruncatedDataYieldsError)
{
  CdrBuilder b;
  b.i32(0);
  b.u32(0);
  b.str("cam");
  b.u32(2);  // height
  b.u32(2);  // width
  b.str("bgr8");
  b.u8(0);
  b.u32(6);       // step
  b.raw_u32(12);  // data length claims 12 bytes, but the buffer ends here
  const auto r = extract_raw_image(b.view());
  EXPECT_FALSE(r.ok());
  EXPECT_FALSE(r.image.has_value());
  EXPECT_FALSE(r.error.empty());
}

TEST(RawImageTest, MalformedPayloadYieldsError)
{
  // A valid header, then an absurd frame_id length that overruns the payload.
  CdrBuilder b;
  b.i32(0);
  b.u32(0);
  b.raw_u32(0xFFFFFFFFU);  // header.frame_id length -> read_string underflows
  const auto r = extract_raw_image(b.view());
  EXPECT_FALSE(r.ok());
}

}  // namespace
