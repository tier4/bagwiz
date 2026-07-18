// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/io/message_decompressor.hpp"

#include <gtest/gtest.h>
#include <zstd.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

// Compress `plain` with libzstd in single-shot mode. The frame header
// includes the original size, matching the layout rosbag2's MESSAGE-mode
// writer emits.
std::vector<std::byte> zstd_compress_frame(std::span<const std::byte> plain)
{
  const std::size_t bound = ZSTD_compressBound(plain.size());
  std::vector<std::byte> compressed(bound);
  const std::size_t written = ZSTD_compress(
    compressed.data(), compressed.size(), plain.data(), plain.size(), /*compressionLevel=*/1);
  EXPECT_FALSE(static_cast<bool>(ZSTD_isError(written))) << ZSTD_getErrorName(written);
  compressed.resize(written);
  return compressed;
}

template <std::size_t N>
std::array<std::byte, N> make_bytes(std::array<std::uint8_t, N> raw)
{
  std::array<std::byte, N> out{};
  for (std::size_t i = 0; i < N; ++i) {
    out.at(i) = std::byte{raw.at(i)};
  }
  return out;
}

}  // namespace

TEST(MessageDecompressorTest, UnsupportedFormatThrows)
{
  EXPECT_THROW(bagwiz::io::MessageDecompressor{"gzip"}, std::runtime_error);
  EXPECT_THROW(bagwiz::io::MessageDecompressor{""}, std::runtime_error);
}

TEST(MessageDecompressorTest, EmptyInputReturnsEmptyOutput)
{
  bagwiz::io::MessageDecompressor decompressor{"zstd"};
  const auto out = decompressor.decompress({});
  EXPECT_TRUE(out.empty());
}

TEST(MessageDecompressorTest, RoundTripsSingleFrame)
{
  // CDR-shaped payload: deterministic, mid-sized, includes the bytes that
  // commonly tripped up the earlier "raw blob" path (0x00, 0xFF, repeats).
  const auto raw = make_bytes(
    std::array<std::uint8_t, 12>{
      0x00, 0x01, 0xDE, 0xAD, 0xBE, 0xEF, 0x42, 0x42, 0x42, 0xFF, 0xFE, 0xFD});
  const auto compressed = zstd_compress_frame({raw.data(), raw.size()});

  bagwiz::io::MessageDecompressor decompressor{"zstd"};
  const auto out = decompressor.decompress({compressed.data(), compressed.size()});

  ASSERT_EQ(out.size(), raw.size());
  for (std::size_t i = 0; i < raw.size(); ++i) {
    EXPECT_EQ(out[i], raw[i]) << "byte " << i << " mismatched";
  }
}

TEST(MessageDecompressorTest, ContextIsReusedAcrossCalls)
{
  // Reusing one decompressor across many calls is the hot-path contract
  // rosbag2 itself relies on (see ZstdDecompressor::ZstdDecompressor's
  // comment about per-thread DCtx). Reuse must not corrupt later outputs.
  bagwiz::io::MessageDecompressor decompressor{"zstd"};

  const auto a = make_bytes(std::array<std::uint8_t, 4>{0x10, 0x20, 0x30, 0x40});
  const auto b = make_bytes(std::array<std::uint8_t, 6>{0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF});

  const auto ca = zstd_compress_frame({a.data(), a.size()});
  const auto cb = zstd_compress_frame({b.data(), b.size()});

  const auto out_a = decompressor.decompress({ca.data(), ca.size()});
  ASSERT_EQ(out_a.size(), a.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(out_a[i], a[i]);
  }

  // The previous span is documented as invalidated by the next call. Trust
  // that contract: only the freshly returned span is allowed to be read.
  const auto out_b = decompressor.decompress({cb.data(), cb.size()});
  ASSERT_EQ(out_b.size(), b.size());
  for (std::size_t i = 0; i < b.size(); ++i) {
    EXPECT_EQ(out_b[i], b[i]);
  }
}

TEST(MessageDecompressorTest, MalformedInputThrows)
{
  bagwiz::io::MessageDecompressor decompressor{"zstd"};
  const auto garbage =
    make_bytes(std::array<std::uint8_t, 8>{0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08});
  EXPECT_THROW((void)decompressor.decompress({garbage.data(), garbage.size()}), std::runtime_error);
}

TEST(MessageDecompressorTest, ReportsFormat)
{
  bagwiz::io::MessageDecompressor decompressor{"zstd"};
  EXPECT_EQ(decompressor.format(), "zstd");
}
