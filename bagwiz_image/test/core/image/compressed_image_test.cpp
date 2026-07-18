// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/image/compressed_image.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace
{
using bagwiz::core::image::extract_compressed_image;

// Little-endian CDR-1 builder, matching the wire format the production reader
// consumes (mirrors the helper in generate_video_test.cpp / raw_image_test.cpp).
class CdrBuilder
{
public:
  CdrBuilder()
  {
    for (int b : {0x00, 0x01, 0x00, 0x00}) {
      buf_.push_back(static_cast<std::byte>(b));
    }
  }
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

// Serialize a sensor_msgs/msg/CompressedImage with the given format string and
// compressed bytes. `stamp_ns` sets header.stamp; the default of 0 leaves it unset.
std::vector<std::byte> make_compressed_payload(
  const std::string & format, std::span<const std::byte> data, std::int64_t stamp_ns = 0)
{
  CdrBuilder b;
  b.i32(static_cast<std::int32_t>(stamp_ns / 1'000'000'000LL));   // header.stamp.sec
  b.u32(static_cast<std::uint32_t>(stamp_ns % 1'000'000'000LL));  // header.stamp.nanosec
  b.str("cam");
  b.str(format);
  b.byte_seq(data);
  return b.take();
}

}  // namespace

TEST(CompressedImageTest, ParsesHeaderStamp)
{
  const std::vector<std::byte> blob{std::byte{0xFF}, std::byte{0xD8}};
  // 1700000000.250000000 s -> 1700000000250000000 ns
  const auto payload =
    make_compressed_payload("jpeg", {blob.data(), blob.size()}, 1'700'000'000'250'000'000LL);
  const auto result = extract_compressed_image({payload.data(), payload.size()});
  ASSERT_TRUE(result.ok()) << result.error;
  EXPECT_EQ(result.image->header_stamp_ns, 1'700'000'000'250'000'000LL);
}

TEST(CompressedImageTest, ParsesFormatAndData)
{
  const std::vector<std::byte> blob{
    std::byte{0xFF}, std::byte{0xD8}, std::byte{0xFF}, std::byte{0xE0}, std::byte{0x42}};
  const auto payload = make_compressed_payload("jpeg", {blob.data(), blob.size()});

  const auto result = extract_compressed_image({payload.data(), payload.size()});
  ASSERT_TRUE(result.ok()) << result.error;
  EXPECT_EQ(result.image->format, "jpeg");
  ASSERT_EQ(result.image->data.size(), blob.size());
  EXPECT_TRUE(std::equal(blob.begin(), blob.end(), result.image->data.begin()));
}

// The image_transport convention puts the source encoding in the format string;
// the parser returns it verbatim without interpreting it.
TEST(CompressedImageTest, PreservesImageTransportFormatString)
{
  const std::vector<std::byte> blob{std::byte{0x89}, std::byte{0x50}};
  const auto payload =
    make_compressed_payload("rgb8; png compressed bgr8", {blob.data(), blob.size()});

  const auto result = extract_compressed_image({payload.data(), payload.size()});
  ASSERT_TRUE(result.ok()) << result.error;
  EXPECT_EQ(result.image->format, "rgb8; png compressed bgr8");
}

// A zero-length data sequence parses cleanly to an empty span (a separate layer
// decides such a frame is undecodable).
TEST(CompressedImageTest, ParsesEmptyDataSequence)
{
  const auto payload = make_compressed_payload("jpeg", {});

  const auto result = extract_compressed_image({payload.data(), payload.size()});
  ASSERT_TRUE(result.ok()) << result.error;
  EXPECT_EQ(result.image->format, "jpeg");
  EXPECT_TRUE(result.image->data.empty());
}

// A payload truncated before the data sequence yields an error, not a throw.
TEST(CompressedImageTest, TruncatedPayloadIsError)
{
  const std::vector<std::byte> blob{std::byte{0xFF}, std::byte{0xD8}};
  auto payload = make_compressed_payload("jpeg", {blob.data(), blob.size()});
  payload.resize(payload.size() - 4);  // chop into the trailing data bytes

  const auto result = extract_compressed_image({payload.data(), payload.size()});
  EXPECT_FALSE(result.ok());
  EXPECT_FALSE(result.error.empty());
}
