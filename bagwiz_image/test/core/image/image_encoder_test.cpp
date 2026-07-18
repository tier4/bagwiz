// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/image/image_encoder.hpp"

#include "bagwiz/core/image/image_decoder.hpp"
#include "bagwiz/core/image/packed_raster.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace
{
using bagwiz::core::image::decode_compressed_image;
using bagwiz::core::image::encode_png;
using bagwiz::core::image::PackedRaster;

// Build a packed BGR24 raster whose every channel is a deterministic function of
// its (x, y, channel) position, so a lossless round-trip can be checked exactly
// without relying on a solid color hiding an off-by-one.
PackedRaster make_gradient(std::uint32_t w, std::uint32_t h)
{
  PackedRaster raster;
  raster.width = w;
  raster.height = h;
  raster.encoding = "bgr8";
  raster.bgr.resize(static_cast<std::size_t>(w) * h * 3);
  for (std::uint32_t y = 0; y < h; ++y) {
    for (std::uint32_t x = 0; x < w; ++x) {
      const std::size_t base = (static_cast<std::size_t>(y) * w + x) * 3;
      raster.bgr[base + 0] = static_cast<std::byte>((x * 7U + y * 3U) & 0xFFU);    // blue
      raster.bgr[base + 1] = static_cast<std::byte>((x * 11U + y * 5U) & 0xFFU);   // green
      raster.bgr[base + 2] = static_cast<std::byte>((x * 13U + y * 17U) & 0xFFU);  // red
    }
  }
  return raster;
}

}  // namespace

// PNG is lossless: encoding a raster and decoding it back must reproduce every
// BGR byte exactly, which is the property walk's "save preview frame" relies on.
TEST(ImageEncoderTest, PngRoundTripsExactly)
{
  const PackedRaster source = make_gradient(8, 6);

  const auto encoded = encode_png(source);
  ASSERT_TRUE(encoded.ok()) << encoded.error;

  const auto & png = *encoded.png;
  const auto decoded = decode_compressed_image({png.data(), png.size()}, "png");
  ASSERT_TRUE(decoded.ok()) << decoded.error;
  EXPECT_EQ(decoded.image->width, source.width);
  EXPECT_EQ(decoded.image->height, source.height);
  ASSERT_EQ(decoded.image->bgr.size(), source.bgr.size());
  EXPECT_EQ(decoded.image->bgr, source.bgr);
}

// The output is a real PNG: it starts with the 8-byte PNG signature.
TEST(ImageEncoderTest, EmitsPngSignature)
{
  const auto encoded = encode_png(make_gradient(4, 4));
  ASSERT_TRUE(encoded.ok()) << encoded.error;

  static constexpr std::array<std::uint8_t, 8> kPngSig{0x89, 0x50, 0x4E, 0x47,
                                                       0x0D, 0x0A, 0x1A, 0x0A};
  ASSERT_GE(encoded.png->size(), kPngSig.size());
  for (std::size_t i = 0; i < kPngSig.size(); ++i) {
    EXPECT_EQ((*encoded.png)[i], static_cast<std::byte>(kPngSig[i])) << "byte " << i;
  }
}

// A 1x1 raster exercises the smallest valid image and the row-stride copy into
// the libav frame buffer, whose aligned linesize far exceeds a one-pixel row.
TEST(ImageEncoderTest, EncodesSinglePixel)
{
  PackedRaster source;
  source.width = 1;
  source.height = 1;
  source.encoding = "bgr8";
  source.bgr = {std::byte{30}, std::byte{20}, std::byte{10}};  // B, G, R

  const auto encoded = encode_png(source);
  ASSERT_TRUE(encoded.ok()) << encoded.error;

  const auto & png = *encoded.png;
  const auto decoded = decode_compressed_image({png.data(), png.size()}, "png");
  ASSERT_TRUE(decoded.ok()) << decoded.error;
  EXPECT_EQ(decoded.image->width, 1U);
  EXPECT_EQ(decoded.image->height, 1U);
  ASSERT_EQ(decoded.image->bgr.size(), 3U);
  EXPECT_EQ(decoded.image->bgr[0], std::byte{30});
  EXPECT_EQ(decoded.image->bgr[1], std::byte{20});
  EXPECT_EQ(decoded.image->bgr[2], std::byte{10});
}

// A 3-pixel-wide raster has a packed row far narrower than the frame buffer's
// aligned linesize; the per-row copy must honor that stride and still round-trip
// rather than reading or writing past the packed rows.
TEST(ImageEncoderTest, EncodesNarrowImage)
{
  const PackedRaster source = make_gradient(3, 5);

  const auto encoded = encode_png(source);
  ASSERT_TRUE(encoded.ok()) << encoded.error;

  const auto & png = *encoded.png;
  const auto decoded = decode_compressed_image({png.data(), png.size()}, "png");
  ASSERT_TRUE(decoded.ok()) << decoded.error;
  ASSERT_EQ(decoded.image->bgr.size(), source.bgr.size());
  EXPECT_EQ(decoded.image->bgr, source.bgr);
}

TEST(ImageEncoderTest, EmptyRasterIsError)
{
  const auto encoded = encode_png(PackedRaster{});
  EXPECT_FALSE(encoded.ok());
  EXPECT_FALSE(encoded.error.empty());
}

// A raster whose pixel buffer does not match width * 3 * height is malformed and
// must be rejected before libav is handed a short buffer.
TEST(ImageEncoderTest, MismatchedBufferSizeIsError)
{
  PackedRaster source;
  source.width = 4;
  source.height = 4;
  source.encoding = "bgr8";
  source.bgr.resize(4U * 4U * 3U - 1U);  // one byte short

  const auto encoded = encode_png(source);
  EXPECT_FALSE(encoded.ok());
  EXPECT_FALSE(encoded.error.empty());
}

// Dimensions beyond what libav's int-typed API can express are rejected up front
// (a tiny pixel buffer keeps the test from allocating a real giant image).
TEST(ImageEncoderTest, OversizeDimensionIsError)
{
  PackedRaster source;
  source.width = static_cast<std::uint32_t>(std::numeric_limits<int>::max()) + 1U;
  source.height = 1;
  source.encoding = "bgr8";
  source.bgr = {std::byte{0}, std::byte{0}, std::byte{0}};

  const auto encoded = encode_png(source);
  EXPECT_FALSE(encoded.ok());
  EXPECT_FALSE(encoded.error.empty());
}
