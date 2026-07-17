// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/image/sampling.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>

namespace bagwiz::core::image
{

std::array<double, 3> bilinear_sample_bgr(
  std::span<const std::byte> bgr, std::uint32_t width, std::uint32_t height, double u, double v)
{
  if (width == 0 || height == 0) {
    return {0.0, 0.0, 0.0};
  }
  const std::size_t num_bytes =
    static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3;
  if (bgr.size() != num_bytes) {
    return {0.0, 0.0, 0.0};
  }
  // NaN has no meaningful clamp target, and converting NaN to an integer below
  // would be undefined behavior.
  if (std::isnan(u) || std::isnan(v)) {
    return {0.0, 0.0, 0.0};
  }

  // Clamp before choosing the 4-tap footprint so that border samples replicate
  // the edge pixel instead of blending with a phantom row/column outside the
  // image.
  const double cu = std::clamp(u, 0.0, static_cast<double>(width - 1));
  const double cv = std::clamp(v, 0.0, static_cast<double>(height - 1));
  const std::uint32_t u0 = static_cast<std::uint32_t>(std::floor(cu));
  const std::uint32_t v0 = static_cast<std::uint32_t>(std::floor(cv));
  const std::uint32_t u1 = std::min(u0 + 1, width - 1);
  const std::uint32_t v1 = std::min(v0 + 1, height - 1);
  const double fu = cu - static_cast<double>(u0);
  const double fv = cv - static_cast<double>(v0);

  const std::size_t row_stride = static_cast<std::size_t>(width) * 3;
  const auto pixel_at = [&](std::uint32_t col, std::uint32_t row, std::size_t channel) {
    const std::size_t i =
      static_cast<std::size_t>(row) * row_stride + static_cast<std::size_t>(col) * 3 + channel;
    return static_cast<double>(bgr[i]);
  };

  std::array<double, 3> sample{};
  for (std::size_t channel = 0; channel < 3; ++channel) {
    const double p00 = pixel_at(u0, v0, channel);
    const double p10 = pixel_at(u1, v0, channel);
    const double p01 = pixel_at(u0, v1, channel);
    const double p11 = pixel_at(u1, v1, channel);
    sample[channel] =
      (1.0 - fu) * (1.0 - fv) * p00 + fu * (1.0 - fv) * p10 + (1.0 - fu) * fv * p01 + fu * fv * p11;
  }
  return sample;
}

}  // namespace bagwiz::core::image
