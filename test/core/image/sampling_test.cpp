// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/image/sampling.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace
{

namespace image = bagwiz::core::image;

// Build a packed BGR24 raster (b, g, r byte order per pixel) from a per-pixel
// callback returning {b, g, r} for column x, row y.
template <typename Fn>
std::vector<std::byte> make_bgr(std::uint32_t width, std::uint32_t height, Fn && fn)
{
  std::vector<std::byte> raster(static_cast<std::size_t>(width) * height * 3);
  for (std::uint32_t y = 0; y < height; ++y) {
    for (std::uint32_t x = 0; x < width; ++x) {
      const auto pixel = fn(x, y);
      const std::size_t i = (static_cast<std::size_t>(y) * width + x) * 3;
      raster[i + 0] = static_cast<std::byte>(pixel[0]);
      raster[i + 1] = static_cast<std::byte>(pixel[1]);
      raster[i + 2] = static_cast<std::byte>(pixel[2]);
    }
  }
  return raster;
}

std::vector<std::byte> make_solid_bgr(
  std::uint32_t width, std::uint32_t height, std::uint8_t b, std::uint8_t g, std::uint8_t r)
{
  return make_bgr(width, height, [&](std::uint32_t, std::uint32_t) {
    return std::array<std::uint8_t, 3>{b, g, r};
  });
}

void expect_color(const std::array<double, 3> & c, double b, double g, double r)
{
  EXPECT_DOUBLE_EQ(c[0], b);
  EXPECT_DOUBLE_EQ(c[1], g);
  EXPECT_DOUBLE_EQ(c[2], r);
}

TEST(BilinearSampleBgr, SolidColorEverywhere)
{
  const auto raster = make_solid_bgr(4, 3, 10, 20, 30);
  expect_color(image::bilinear_sample_bgr(raster, 4, 3, 0.0, 0.0), 10.0, 20.0, 30.0);
  expect_color(image::bilinear_sample_bgr(raster, 4, 3, 1.5, 1.5), 10.0, 20.0, 30.0);
  expect_color(image::bilinear_sample_bgr(raster, 4, 3, 3.9, 2.9), 10.0, 20.0, 30.0);
  // Out-of-range coordinates clamp into the image and still hit the solid color.
  expect_color(image::bilinear_sample_bgr(raster, 4, 3, -5.0, 100.0), 10.0, 20.0, 30.0);
}

TEST(BilinearSampleBgr, IntegerCoordinatesHitExactPixels)
{
  // 3x2 raster where the b channel encodes 10 * (x + 3 * y), so every pixel differs.
  const auto raster = make_bgr(3, 2, [](std::uint32_t x, std::uint32_t y) {
    const auto base = static_cast<std::uint8_t>(10 * (x + 3 * y));
    return std::array<std::uint8_t, 3>{
      base, static_cast<std::uint8_t>(base + 1), static_cast<std::uint8_t>(base + 2)};
  });
  for (std::uint32_t y = 0; y < 2; ++y) {
    for (std::uint32_t x = 0; x < 3; ++x) {
      const double base = 10.0 * (x + 3 * y);
      expect_color(
        image::bilinear_sample_bgr(raster, 3, 2, static_cast<double>(x), static_cast<double>(y)),
        base, base + 1.0, base + 2.0);
    }
  }
}

TEST(BilinearSampleBgr, MidpointOfBlackWhiteStepGivesHalfMix)
{
  // 2x1 raster: black pixel left, white pixel right.
  const auto raster = make_bgr(2, 1, [](std::uint32_t x, std::uint32_t) {
    const auto v = static_cast<std::uint8_t>(x == 0 ? 0 : 255);
    return std::array<std::uint8_t, 3>{v, v, v};
  });
  expect_color(image::bilinear_sample_bgr(raster, 2, 1, 0.5, 0.0), 127.5, 127.5, 127.5);
}

TEST(BilinearSampleBgr, SinglePixelImage)
{
  const auto raster = make_solid_bgr(1, 1, 5, 6, 7);
  expect_color(image::bilinear_sample_bgr(raster, 1, 1, 0.0, 0.0), 5.0, 6.0, 7.0);
  expect_color(image::bilinear_sample_bgr(raster, 1, 1, 0.9, 0.9), 5.0, 6.0, 7.0);
  expect_color(image::bilinear_sample_bgr(raster, 1, 1, -3.0, 8.0), 5.0, 6.0, 7.0);
}

TEST(BilinearSampleBgr, ClampsToEdgePixelsAtAllBorders)
{
  // Distinct per-pixel values: b = 40x + 7y, g = 10x, r = 30y. Each out-of-range
  // coordinate must sample the same pixel as the coordinate clamped onto the
  // image boundary.
  const auto raster = make_bgr(3, 3, [](std::uint32_t x, std::uint32_t y) {
    return std::array<std::uint8_t, 3>{
      static_cast<std::uint8_t>(40 * x + 7 * y), static_cast<std::uint8_t>(10 * x),
      static_cast<std::uint8_t>(30 * y)};
  });
  expect_color(image::bilinear_sample_bgr(raster, 3, 3, -1.5, 1.0), 7.0, 0.0, 30.0);   // left
  expect_color(image::bilinear_sample_bgr(raster, 3, 3, 2.9, 1.0), 87.0, 20.0, 30.0);  // right
  expect_color(image::bilinear_sample_bgr(raster, 3, 3, 1.0, -0.5), 40.0, 10.0, 0.0);  // top
  expect_color(image::bilinear_sample_bgr(raster, 3, 3, 1.0, 2.7), 54.0, 10.0, 60.0);  // bottom
  expect_color(image::bilinear_sample_bgr(raster, 3, 3, -1.0, -1.0), 0.0, 0.0, 0.0);   // top-left
  expect_color(image::bilinear_sample_bgr(raster, 3, 3, 9.0, -9.0), 80.0, 20.0, 0.0);  // top-right
  expect_color(
    image::bilinear_sample_bgr(raster, 3, 3, -9.0, 9.0), 14.0, 0.0, 60.0);  // bottom-left
  expect_color(
    image::bilinear_sample_bgr(raster, 3, 3, 9.0, 9.0), 94.0, 20.0, 60.0);  // bottom-right
}

TEST(BilinearSampleBgr, LinearRampMatchesClosedForm)
{
  // Exact linear ramps in x, constant in y: b = 8x, g = 200 - 4x, r = 17 + 6x.
  // Bilinear interpolation of an exactly linear field must reproduce the field
  // at any subpixel position.
  constexpr std::uint32_t kWidth = 9;
  constexpr std::uint32_t kHeight = 5;
  const auto raster = make_bgr(kWidth, kHeight, [](std::uint32_t x, std::uint32_t) {
    return std::array<std::uint8_t, 3>{
      static_cast<std::uint8_t>(8 * x), static_cast<std::uint8_t>(200 - 4 * x),
      static_cast<std::uint8_t>(17 + 6 * x)};
  });
  constexpr double kTol = 1e-9;
  for (const double u : {0.25, 1.5, 3.75, 7.2, 8.0}) {
    const auto c = image::bilinear_sample_bgr(raster, kWidth, kHeight, u, 2.3);
    EXPECT_NEAR(c[0], 8.0 * u, kTol);
    EXPECT_NEAR(c[1], 200.0 - 4.0 * u, kTol);
    EXPECT_NEAR(c[2], 17.0 + 6.0 * u, kTol);
  }
}

TEST(BilinearSampleBgr, InvalidInputReturnsZero)
{
  const auto raster = make_solid_bgr(2, 2, 9, 8, 7);
  expect_color(image::bilinear_sample_bgr(raster, 0, 2, 0.0, 0.0), 0.0, 0.0, 0.0);
  expect_color(image::bilinear_sample_bgr(raster, 2, 0, 0.0, 0.0), 0.0, 0.0, 0.0);
  // Buffer smaller than width*height*3 bytes.
  expect_color(
    image::bilinear_sample_bgr(std::span<const std::byte>(raster.data(), 6), 2, 2, 0.5, 0.5), 0.0,
    0.0, 0.0);
  // Buffer larger than width*height*3 bytes (a packed raster has an exact size).
  expect_color(
    image::bilinear_sample_bgr(std::span<const std::byte>(raster.data(), 12), 1, 2, 0.0, 0.0), 0.0,
    0.0, 0.0);
  expect_color(image::bilinear_sample_bgr({}, 2, 2, 0.0, 0.0), 0.0, 0.0, 0.0);
}

}  // namespace
