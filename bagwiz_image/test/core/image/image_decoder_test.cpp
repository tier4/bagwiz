// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/image/image_decoder.hpp"

#include "core/image/image_fixture.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace
{
using bagwiz::core::image::decode_compressed_image;
using bagwiz::test::encode_still_image;

// Channel value at pixel (x, y) of a packed BGR24 raster; c=0 is blue, 1 green,
// 2 red.
int bgr_at(
  const bagwiz::core::image::DecodedImage & img, std::uint32_t x, std::uint32_t y, std::size_t c)
{
  const std::size_t idx = (static_cast<std::size_t>(y) * img.width + x) * 3 + c;
  return static_cast<int>(img.bgr[idx]);
}

}  // namespace

// PNG is lossless, so a solid RGB color round-trips to BGR exactly.
TEST(ImageDecoderTest, DecodesPngToBgrExactly)
{
  const auto png = encode_still_image("png", 8, 8, /*r=*/10, /*g=*/20, /*b=*/30);
  ASSERT_FALSE(png.empty()) << "PNG encoder unavailable";

  const auto result = decode_compressed_image({png.data(), png.size()}, "png");
  ASSERT_TRUE(result.ok()) << result.error;
  EXPECT_EQ(result.image->width, 8U);
  EXPECT_EQ(result.image->height, 8U);
  ASSERT_EQ(result.image->bgr.size(), 8U * 8U * 3U);

  // Channels are stored B, G, R — so the red=10/green=20/blue=30 source lands as
  // 30, 20, 10.
  EXPECT_EQ(bgr_at(*result.image, 0, 0, 0), 30);  // blue
  EXPECT_EQ(bgr_at(*result.image, 0, 0, 1), 20);  // green
  EXPECT_EQ(bgr_at(*result.image, 0, 0, 2), 10);  // red
  EXPECT_EQ(bgr_at(*result.image, 7, 7, 0), 30);  // last pixel is the same solid color
}

// JPEG is lossy; a solid color survives within a small tolerance, and the
// reported geometry is exact.
TEST(ImageDecoderTest, DecodesJpegToBgrApproximately)
{
  const auto jpeg = encode_still_image("jpeg", 16, 16, /*r=*/200, /*g=*/100, /*b=*/50);
  ASSERT_FALSE(jpeg.empty()) << "MJPEG encoder unavailable";

  const auto result = decode_compressed_image({jpeg.data(), jpeg.size()}, "jpeg");
  ASSERT_TRUE(result.ok()) << result.error;
  EXPECT_EQ(result.image->width, 16U);
  EXPECT_EQ(result.image->height, 16U);
  ASSERT_EQ(result.image->bgr.size(), 16U * 16U * 3U);

  EXPECT_NEAR(bgr_at(*result.image, 8, 8, 0), 50, 12);   // blue
  EXPECT_NEAR(bgr_at(*result.image, 8, 8, 1), 100, 12);  // green
  EXPECT_NEAR(bgr_at(*result.image, 8, 8, 2), 200, 12);  // red
}

// The codec is detected from magic bytes regardless of a missing/empty format
// string.
TEST(ImageDecoderTest, DetectsCodecFromMagicWithoutFormatHint)
{
  const auto jpeg = encode_still_image("jpeg", 8, 8, 0, 0, 0);
  ASSERT_FALSE(jpeg.empty());

  const auto result = decode_compressed_image({jpeg.data(), jpeg.size()});
  ASSERT_TRUE(result.ok()) << result.error;
  EXPECT_EQ(result.image->width, 8U);
}

TEST(ImageDecoderTest, EmptyInputIsError)
{
  const auto result = decode_compressed_image({});
  EXPECT_FALSE(result.ok());
  EXPECT_FALSE(result.error.empty());
}

// Bytes that match neither the JPEG nor PNG signature are rejected before any
// decode is attempted.
TEST(ImageDecoderTest, UnrecognizedFormatIsError)
{
  const std::vector<std::byte> garbage{std::byte{0x00}, std::byte{0x01}, std::byte{0x02},
                                       std::byte{0x03}, std::byte{0x04}, std::byte{0x05},
                                       std::byte{0x06}, std::byte{0x07}};

  const auto result = decode_compressed_image({garbage.data(), garbage.size()}, "weird");
  EXPECT_FALSE(result.ok());
  EXPECT_FALSE(result.error.empty());
}

// A JPEG signature followed by a corrupt bitstream fails in the decoder rather
// than producing a frame.
TEST(ImageDecoderTest, CorruptJpegIsError)
{
  const std::vector<std::byte> corrupt{std::byte{0xFF}, std::byte{0xD8}, std::byte{0xFF},
                                       std::byte{0xAA}, std::byte{0xBB}, std::byte{0xCC},
                                       std::byte{0xDD}, std::byte{0xEE}};

  const auto result = decode_compressed_image({corrupt.data(), corrupt.size()}, "jpeg");
  EXPECT_FALSE(result.ok());
  EXPECT_FALSE(result.error.empty());
}

TEST(ImageDecoderTest, ReusedDecoderMatchesFreeFunction)
{
  const auto jpeg = encode_still_image("jpeg", 16, 16, 200, 100, 50);
  ASSERT_FALSE(jpeg.empty());
  bagwiz::core::image::ImageDecoder decoder;
  const auto first = decoder.decode({jpeg.data(), jpeg.size()}, "jpeg");
  const auto second = decoder.decode({jpeg.data(), jpeg.size()}, "jpeg");
  const auto reference = decode_compressed_image({jpeg.data(), jpeg.size()}, "jpeg");
  ASSERT_TRUE(first.ok()) << first.error;
  ASSERT_TRUE(second.ok()) << second.error;
  ASSERT_TRUE(reference.ok()) << reference.error;
  EXPECT_EQ(first.image->bgr, second.image->bgr);     // no state leak frame-to-frame
  EXPECT_EQ(first.image->bgr, reference.image->bgr);  // same pixels as the one-shot path
}

TEST(ImageDecoderTest, ReusedDecoderHandlesSizeChange)
{
  const auto big = encode_still_image("jpeg", 32, 32, 10, 20, 30);
  const auto small = encode_still_image("jpeg", 16, 8, 10, 20, 30);
  ASSERT_FALSE(big.empty());
  ASSERT_FALSE(small.empty());
  bagwiz::core::image::ImageDecoder decoder;
  const auto r1 = decoder.decode({big.data(), big.size()});
  const auto r2 = decoder.decode({small.data(), small.size()});
  ASSERT_TRUE(r1.ok()) << r1.error;
  ASSERT_TRUE(r2.ok()) << r2.error;
  EXPECT_EQ(r1.image->width, 32U);
  EXPECT_EQ(r2.image->width, 16U);
  EXPECT_EQ(r2.image->height, 8U);
  EXPECT_EQ(r2.image->bgr.size(), 16U * 8U * 3U);
}

TEST(ImageDecoderTest, ReusedDecoderSwitchesCodecs)
{
  const auto jpeg = encode_still_image("jpeg", 8, 8, 0, 0, 0);
  const auto png = encode_still_image("png", 8, 8, 10, 20, 30);
  ASSERT_FALSE(jpeg.empty());
  ASSERT_FALSE(png.empty());
  bagwiz::core::image::ImageDecoder decoder;
  EXPECT_TRUE(decoder.decode({jpeg.data(), jpeg.size()}).ok());
  const auto png_result = decoder.decode({png.data(), png.size()});
  ASSERT_TRUE(png_result.ok()) << png_result.error;
  EXPECT_EQ(static_cast<int>(png_result.image->bgr[0]), 30);  // blue channel exact for PNG
}

TEST(ImageDecoderTest, ReusedDecoderRecoversAfterCorruptInput)
{
  const auto good = encode_still_image("jpeg", 8, 8, 0, 0, 0);
  ASSERT_FALSE(good.empty());
  const std::vector<std::byte> corrupt{std::byte{0xFF}, std::byte{0xD8}, std::byte{0xFF},
                                       std::byte{0xAA}, std::byte{0xBB}, std::byte{0xCC}};
  bagwiz::core::image::ImageDecoder decoder;
  EXPECT_TRUE(decoder.decode({good.data(), good.size()}).ok());
  EXPECT_FALSE(decoder.decode({corrupt.data(), corrupt.size()}).ok());
  EXPECT_TRUE(decoder.decode({good.data(), good.size()}).ok());  // context still usable
}

TEST(ImageDecoderTest, DecodeToYuvExposesLumaPlane)
{
  const auto jpeg = encode_still_image("jpeg", 16, 16, 200, 100, 50);
  ASSERT_FALSE(jpeg.empty());
  bagwiz::core::image::ImageDecoder decoder;
  const auto result = decoder.decode_to_yuv({jpeg.data(), jpeg.size()}, "jpeg");
  ASSERT_TRUE(result.ok()) << result.error;
  EXPECT_EQ(result.view->width, 16U);
  EXPECT_EQ(result.view->height, 16U);
  ASSERT_NE(result.view->y, nullptr);
  // BT.601 luma of (r=200, g=100, b=50) ≈ 0.299*200 + 0.587*100 + 0.114*50 ≈ 124.
  EXPECT_NEAR(static_cast<int>(result.view->y[0]), 124, 12);
}

TEST(ImageDecoderTest, SampleRgbMatchesBgrDecode)
{
  const auto jpeg = encode_still_image("jpeg", 16, 16, 200, 100, 50);
  ASSERT_FALSE(jpeg.empty());
  bagwiz::core::image::ImageDecoder decoder;
  const auto yuv = decoder.decode_to_yuv({jpeg.data(), jpeg.size()});
  ASSERT_TRUE(yuv.ok()) << yuv.error;
  const auto rgb = bagwiz::core::image::sample_rgb(*yuv.view, 8, 8);
  const auto bgr = decode_compressed_image({jpeg.data(), jpeg.size()});
  ASSERT_TRUE(bgr.ok());
  // Compare against the swscale-converted pixel at (8, 8): r/g/b within ±4 LSB
  // (independent fixed-point pipelines).
  EXPECT_NEAR(rgb[0], bgr_at(*bgr.image, 8, 8, 2), 4);
  EXPECT_NEAR(rgb[1], bgr_at(*bgr.image, 8, 8, 1), 4);
  EXPECT_NEAR(rgb[2], bgr_at(*bgr.image, 8, 8, 0), 4);
}

TEST(ImageDecoderTest, DecodeToYuvRejectsPng)
{
  const auto png = encode_still_image("png", 8, 8, 10, 20, 30);
  ASSERT_FALSE(png.empty());
  bagwiz::core::image::ImageDecoder decoder;
  const auto result = decoder.decode_to_yuv({png.data(), png.size()}, "png");
  EXPECT_FALSE(result.ok());  // PNG decodes to RGB — caller falls back to decode()
  EXPECT_FALSE(result.error.empty());
}
