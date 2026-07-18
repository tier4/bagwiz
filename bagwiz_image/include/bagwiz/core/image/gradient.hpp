// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__IMAGE__GRADIENT_HPP_
#define BAGWIZ__CORE__IMAGE__GRADIENT_HPP_

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace bagwiz::core::image
{

// Per-pixel gradient magnitude of a packed BGR24 raster: luma grayscale
// (0.114 b + 0.587 g + 0.299 r) convolved with the standard 3x3 Sobel kernels,
// magnitude |gx| + |gy| (kernel sums NOT normalized, i.e. a full 0->255 step edge
// across one pixel gives 4*255 = 1020). Border pixels replicate the adjacent
// interior value; images smaller than 3x3 have no interior pixels and yield all
// zeros. Returns width*height floats, empty on invalid input (zero dimensions or
// a buffer that does not hold exactly width*height*3 bytes).
[[nodiscard]] std::vector<float> sobel_gradient_magnitude(
  std::span<const std::byte> bgr, std::uint32_t width, std::uint32_t height);

// The same gradient magnitude bilinearly sampled at subpixel (u, v) (u =
// column, v = row, integer values land on pixel centers), computed LAZILY:
// only the four corner neighborhoods are evaluated instead of the whole map,
// so callers that need a handful of samples per image skip the full-image
// pass. Produces the same values as bilinear-sampling
// sobel_gradient_magnitude() with clamped coordinates; 0.0 on invalid input
// or NaN coordinates.
[[nodiscard]] double sobel_gradient_magnitude_bilinear(
  std::span<const std::byte> bgr, std::uint32_t width, std::uint32_t height, double u, double v);

}  // namespace bagwiz::core::image

#endif  // BAGWIZ__CORE__IMAGE__GRADIENT_HPP_
