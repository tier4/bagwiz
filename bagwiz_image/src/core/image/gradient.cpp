// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/image/gradient.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace bagwiz::core::image
{

namespace
{

float luma_at(const std::byte * bgr, std::size_t pixel)
{
  const double b = static_cast<double>(bgr[pixel * 3 + 0]);
  const double g = static_cast<double>(bgr[pixel * 3 + 1]);
  const double r = static_cast<double>(bgr[pixel * 3 + 2]);
  return static_cast<float>(0.114 * b + 0.587 * g + 0.299 * r);
}

// Sobel magnitude at one pixel, border pixels evaluated at the clamped
// interior coordinate (matching the full-map border replication). Only valid
// for width >= 3 and height >= 3.
float sobel_at(
  const std::byte * bgr, std::size_t width, std::size_t height, std::size_t col, std::size_t row)
{
  col = std::clamp(col, std::size_t{1}, width - 2);
  row = std::clamp(row, std::size_t{1}, height - 2);
  const auto g = [&](std::size_t c, std::size_t r) { return luma_at(bgr, r * width + c); };
  const float gx = (g(col + 1, row - 1) + 2.0F * g(col + 1, row) + g(col + 1, row + 1)) -
                   (g(col - 1, row - 1) + 2.0F * g(col - 1, row) + g(col - 1, row + 1));
  const float gy = (g(col - 1, row + 1) + 2.0F * g(col, row + 1) + g(col + 1, row + 1)) -
                   (g(col - 1, row - 1) + 2.0F * g(col, row - 1) + g(col + 1, row - 1));
  return std::fabs(gx) + std::fabs(gy);
}

}  // namespace

std::vector<float> sobel_gradient_magnitude(
  std::span<const std::byte> bgr, std::uint32_t width, std::uint32_t height)
{
  if (width == 0 || height == 0) {
    return {};
  }
  const std::size_t num_pixels = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  if (bgr.size() != num_pixels * 3) {
    return {};
  }

  // Luma grayscale pass (0.114 b + 0.587 g + 0.299 r).
  std::vector<float> gray(num_pixels);
  for (std::size_t i = 0; i < num_pixels; ++i) {
    const double b = static_cast<double>(bgr[i * 3 + 0]);
    const double g = static_cast<double>(bgr[i * 3 + 1]);
    const double r = static_cast<double>(bgr[i * 3 + 2]);
    gray[i] = static_cast<float>(0.114 * b + 0.587 * g + 0.299 * r);
  }

  // Images below 3x3 have no pixel with a complete 3x3 neighborhood, so there
  // is no interior to convolve and the gradient is zero everywhere.
  std::vector<float> magnitude(num_pixels, 0.0F);
  if (width < 3 || height < 3) {
    return magnitude;
  }

  // Sobel pass over the interior. The kernel sums are intentionally left
  // unnormalized, so a full 0 -> 255 step across one pixel reads 4 * 255 = 1020.
  const std::size_t w = width;
  const std::size_t h = height;
  const auto gray_at = [&](std::size_t col, std::size_t row) { return gray[row * w + col]; };
  for (std::size_t row = 1; row + 1 < h; ++row) {
    for (std::size_t col = 1; col + 1 < w; ++col) {
      const float gx =
        (gray_at(col + 1, row - 1) + 2.0F * gray_at(col + 1, row) + gray_at(col + 1, row + 1)) -
        (gray_at(col - 1, row - 1) + 2.0F * gray_at(col - 1, row) + gray_at(col - 1, row + 1));
      const float gy =
        (gray_at(col - 1, row + 1) + 2.0F * gray_at(col, row + 1) + gray_at(col + 1, row + 1)) -
        (gray_at(col - 1, row - 1) + 2.0F * gray_at(col, row - 1) + gray_at(col + 1, row - 1));
      magnitude[row * w + col] = std::fabs(gx) + std::fabs(gy);
    }
  }

  // Border pixels are not convolved; they replicate the adjacent interior value
  // (corners replicate the adjacent interior corner).
  for (std::size_t col = 0; col < w; ++col) {
    const std::size_t interior_col = std::clamp(col, std::size_t{1}, w - 2);
    magnitude[col] = magnitude[w + interior_col];
    magnitude[(h - 1) * w + col] = magnitude[(h - 2) * w + interior_col];
  }
  for (std::size_t row = 0; row < h; ++row) {
    const std::size_t interior_row = std::clamp(row, std::size_t{1}, h - 2);
    magnitude[row * w] = magnitude[interior_row * w + 1];
    magnitude[row * w + (w - 1)] = magnitude[interior_row * w + (w - 2)];
  }
  return magnitude;
}

double sobel_gradient_magnitude_bilinear(
  std::span<const std::byte> bgr, std::uint32_t width, std::uint32_t height, double u, double v)
{
  if (
    width == 0 || height == 0 ||
    bgr.size() != static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3) {
    return 0.0;
  }
  if (width < 3 || height < 3) {
    return 0.0;  // no interior pixels, mirroring the full-map result
  }
  if (std::isnan(u) || std::isnan(v)) {
    return 0.0;
  }
  const double cu = std::clamp(u, 0.0, static_cast<double>(width - 1));
  const double cv = std::clamp(v, 0.0, static_cast<double>(height - 1));
  const std::uint32_t u0 = static_cast<std::uint32_t>(cu);
  const std::uint32_t v0 = static_cast<std::uint32_t>(cv);
  const std::uint32_t u1 = std::min(u0 + 1, width - 1);
  const std::uint32_t v1 = std::min(v0 + 1, height - 1);
  const double fu = cu - static_cast<double>(u0);
  const double fv = cv - static_cast<double>(v0);
  const std::byte * data = bgr.data();
  const double m00 = sobel_at(data, width, height, u0, v0);
  const double m01 = sobel_at(data, width, height, u1, v0);
  const double m10 = sobel_at(data, width, height, u0, v1);
  const double m11 = sobel_at(data, width, height, u1, v1);
  return (1.0 - fu) * (1.0 - fv) * m00 + fu * (1.0 - fv) * m01 + (1.0 - fu) * fv * m10 +
         fu * fv * m11;
}

}  // namespace bagwiz::core::image
