// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__IMAGE__SAMPLING_HPP_
#define BAGWIZ__CORE__IMAGE__SAMPLING_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace bagwiz::core::image
{

// Bilinear sample of a packed BGR24 raster at subpixel position (u, v), in pixel
// coordinates (u = column, v = row, integer values land on pixel centers). Returns
// {b, g, r} in the raster's channel order. Coordinates are clamped so the 4-tap
// footprint stays inside the image. Returns {0,0,0} for empty/invalid input
// (zero dimensions, NaN coordinates, or a buffer that does not hold exactly
// width*height*3 bytes).
[[nodiscard]] std::array<double, 3> bilinear_sample_bgr(
  std::span<const std::byte> bgr, std::uint32_t width, std::uint32_t height, double u, double v);

}  // namespace bagwiz::core::image

#endif  // BAGWIZ__CORE__IMAGE__SAMPLING_HPP_
