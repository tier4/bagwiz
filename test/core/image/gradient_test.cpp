// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/image/gradient.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
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

TEST(SobelGradientMagnitude, FlatRasterIsZero)
{
  const auto raster = make_solid_bgr(6, 5, 30, 200, 90);
  const auto grad = image::sobel_gradient_magnitude(raster, 6, 5);
  ASSERT_EQ(grad.size(), 30U);
  for (const float v : grad) {
    EXPECT_FLOAT_EQ(v, 0.0F);
  }
}

TEST(SobelGradientMagnitude, VerticalStepEdgePeaksAtEdge)
{
  // Left half black, right half white: a full 0 -> 255 luma step, which the
  // unnormalized Sobel kernels measure as |gx| + |gy| = 4 * 255 = 1020 on both
  // sides of the edge.
  constexpr std::uint32_t kWidth = 8;
  constexpr std::uint32_t kHeight = 6;
  const auto raster = make_bgr(kWidth, kHeight, [](std::uint32_t x, std::uint32_t) {
    const auto v = static_cast<std::uint8_t>(x < 4 ? 0 : 255);
    return std::array<std::uint8_t, 3>{v, v, v};
  });
  const auto grad = image::sobel_gradient_magnitude(raster, kWidth, kHeight);
  ASSERT_EQ(grad.size(), static_cast<std::size_t>(kWidth) * kHeight);
  const auto at = [&](std::uint32_t col, std::uint32_t row) {
    return grad[static_cast<std::size_t>(row) * kWidth + col];
  };
  for (std::uint32_t y = 0; y < kHeight; ++y) {
    EXPECT_FLOAT_EQ(at(3, y), 1020.0F);
    EXPECT_FLOAT_EQ(at(4, y), 1020.0F);
    EXPECT_FLOAT_EQ(at(0, y), 0.0F);
    EXPECT_FLOAT_EQ(at(1, y), 0.0F);
    EXPECT_FLOAT_EQ(at(2, y), 0.0F);
    EXPECT_FLOAT_EQ(at(5, y), 0.0F);
    EXPECT_FLOAT_EQ(at(6, y), 0.0F);
    EXPECT_FLOAT_EQ(at(7, y), 0.0F);
  }
}

TEST(SobelGradientMagnitude, BrightImpulseHasNonzeroRing)
{
  // A single bright pixel on black. Each of its 8 neighbors sees the impulse
  // through kernel taps totaling magnitude 2 (orthogonal neighbor: one tap of
  // magnitude 2; diagonal neighbor: two taps of magnitude 1 each, split between
  // gx and gy), so the whole ring reads 2 * 255 = 510.
  constexpr std::uint32_t kWidth = 7;
  constexpr std::uint32_t kHeight = 7;
  const auto raster = make_bgr(kWidth, kHeight, [](std::uint32_t x, std::uint32_t y) {
    const auto v = static_cast<std::uint8_t>(x == 3 && y == 3 ? 255 : 0);
    return std::array<std::uint8_t, 3>{v, v, v};
  });
  const auto grad = image::sobel_gradient_magnitude(raster, kWidth, kHeight);
  ASSERT_EQ(grad.size(), static_cast<std::size_t>(kWidth) * kHeight);
  const auto at = [&](std::uint32_t col, std::uint32_t row) {
    return grad[static_cast<std::size_t>(row) * kWidth + col];
  };
  for (std::uint32_t y = 2; y <= 4; ++y) {
    for (std::uint32_t x = 2; x <= 4; ++x) {
      if (x == 3 && y == 3) {
        // The impulse sits on the zero-weight center tap of both kernels.
        EXPECT_FLOAT_EQ(at(x, y), 0.0F);
      } else {
        EXPECT_FLOAT_EQ(at(x, y), 510.0F);
      }
    }
  }
  // Pixels whose 3x3 footprint never touches the impulse stay zero.
  EXPECT_FLOAT_EQ(at(1, 1), 0.0F);
  EXPECT_FLOAT_EQ(at(5, 5), 0.0F);
  EXPECT_FLOAT_EQ(at(1, 5), 0.0F);
  EXPECT_FLOAT_EQ(at(5, 1), 0.0F);
}

TEST(SobelGradientMagnitude, BorderReplicatesAdjacentInterior)
{
  // Horizontal luma ramp: every interior pixel has the same |gx|, so each border
  // pixel must equal its adjacent interior pixel and the whole map is constant.
  // (If the kernel were instead evaluated on the border with clamped taps, the
  // border columns would read half the interior value.)
  constexpr std::uint32_t kWidth = 8;
  constexpr std::uint32_t kHeight = 6;
  const auto raster = make_bgr(kWidth, kHeight, [](std::uint32_t x, std::uint32_t) {
    const auto v = static_cast<std::uint8_t>(20 * x);
    return std::array<std::uint8_t, 3>{v, v, v};
  });
  const auto grad = image::sobel_gradient_magnitude(raster, kWidth, kHeight);
  ASSERT_EQ(grad.size(), static_cast<std::size_t>(kWidth) * kHeight);
  for (const float v : grad) {
    EXPECT_FLOAT_EQ(v, 160.0F);
  }
}

TEST(SobelGradientMagnitude, ImageSmallerThan3x3YieldsZero)
{
  // Below 3x3 no pixel has a complete 3x3 neighborhood, so the gradient is zero.
  const auto raster = make_bgr(2, 2, [](std::uint32_t x, std::uint32_t) {
    const auto v = static_cast<std::uint8_t>(x == 0 ? 0 : 255);
    return std::array<std::uint8_t, 3>{v, v, v};
  });
  const auto grad = image::sobel_gradient_magnitude(raster, 2, 2);
  ASSERT_EQ(grad.size(), 4U);
  for (const float v : grad) {
    EXPECT_FLOAT_EQ(v, 0.0F);
  }
}

TEST(SobelGradientMagnitude, OutputSizeMatchesImage)
{
  const auto raster = make_solid_bgr(4, 3, 1, 2, 3);
  const auto grad = image::sobel_gradient_magnitude(raster, 4, 3);
  EXPECT_EQ(grad.size(), 12U);
}

TEST(SobelGradientMagnitude, InvalidInputReturnsEmpty)
{
  const auto raster = make_solid_bgr(2, 2, 1, 2, 3);
  EXPECT_TRUE(image::sobel_gradient_magnitude(raster, 0, 2).empty());
  EXPECT_TRUE(image::sobel_gradient_magnitude(raster, 2, 0).empty());
  // Buffer smaller than width*height*3 bytes.
  EXPECT_TRUE(
    image::sobel_gradient_magnitude(std::span<const std::byte>(raster.data(), 6), 2, 3).empty());
  // Buffer larger than width*height*3 bytes (a packed raster has an exact size).
  EXPECT_TRUE(
    image::sobel_gradient_magnitude(std::span<const std::byte>(raster.data(), 12), 1, 2).empty());
  EXPECT_TRUE(image::sobel_gradient_magnitude({}, 2, 2).empty());
}

TEST(SobelGradientMagnitudeBilinear, MatchesFullMapSampling)
{
  // The lazy sampler must agree with computing the full map and bilinear-
  // sampling it, at interior, border, and out-of-range positions.
  const std::uint32_t w = 37;
  const std::uint32_t h = 23;
  const auto raster = make_bgr(w, h, [](std::uint32_t x, std::uint32_t y) {
    const auto v = static_cast<std::uint8_t>((x * 37u + y * 91u + (x * y) % 17u) % 256u);
    return std::array<std::uint8_t, 3>{v, static_cast<std::uint8_t>(255u - v), v};
  });
  const auto map = image::sobel_gradient_magnitude(raster, w, h);
  ASSERT_EQ(map.size(), static_cast<std::size_t>(w) * h);

  auto reference = [&](double u, double v) {
    const double cu = std::clamp(u, 0.0, static_cast<double>(w - 1));
    const double cv = std::clamp(v, 0.0, static_cast<double>(h - 1));
    const std::uint32_t u0 = static_cast<std::uint32_t>(cu);
    const std::uint32_t v0 = static_cast<std::uint32_t>(cv);
    const std::uint32_t u1 = std::min(u0 + 1, w - 1);
    const std::uint32_t v1 = std::min(v0 + 1, h - 1);
    const double fu = cu - static_cast<double>(u0);
    const double fv = cv - static_cast<double>(v0);
    const double m00 = map[static_cast<std::size_t>(v0) * w + u0];
    const double m01 = map[static_cast<std::size_t>(v0) * w + u1];
    const double m10 = map[static_cast<std::size_t>(v1) * w + u0];
    const double m11 = map[static_cast<std::size_t>(v1) * w + u1];
    return (1.0 - fu) * (1.0 - fv) * m00 + fu * (1.0 - fv) * m01 + (1.0 - fu) * fv * m10 +
           fu * fv * m11;
  };

  const double positions[][2] = {
    {0.0, 0.0}, {5.25, 7.75}, {18.5, 11.5},   {36.0, 22.0}, {36.9, 22.9},
    {0.4, 0.6}, {1.0, 1.0},   {35.99, 21.99}, {-3.0, 10.0}, {50.0, -1.0},
  };
  for (const auto & [u, v] : positions) {
    EXPECT_NEAR(image::sobel_gradient_magnitude_bilinear(raster, w, h, u, v), reference(u, v), 1e-4)
      << "at (" << u << ", " << v << ")";
  }
}

TEST(SobelGradientMagnitudeBilinear, DegenerateInputsAreZero)
{
  const auto raster = make_solid_bgr(4, 3, 1, 2, 3);
  EXPECT_DOUBLE_EQ(image::sobel_gradient_magnitude_bilinear(raster, 0, 3, 1.0, 1.0), 0.0);
  EXPECT_DOUBLE_EQ(image::sobel_gradient_magnitude_bilinear(raster, 2, 3, 1.0, 1.0), 0.0);
  EXPECT_DOUBLE_EQ(
    image::sobel_gradient_magnitude_bilinear(
      std::span<const std::byte>(raster.data(), 6), 2, 3, 1.0, 1.0),
    0.0);
  EXPECT_DOUBLE_EQ(image::sobel_gradient_magnitude_bilinear(raster, 4, 3, NAN, 1.0), 0.0);
  // A flat raster has zero gradient anywhere, including lazily.
  EXPECT_DOUBLE_EQ(image::sobel_gradient_magnitude_bilinear(raster, 4, 3, 1.5, 1.5), 0.0);
}

}  // namespace
