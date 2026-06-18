// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SLAM__POINT_CLOUD_IO_HPP_
#define BAGWIZ__CORE__SLAM__POINT_CLOUD_IO_HPP_

#include <array>
#include <cstdint>
#include <ostream>
#include <span>

// GLIM-free point-cloud writers used by `bagwiz slam --map`. Kept free of GLIM /
// Eigen / gtsam_points types so the writer (and its test) build in every
// configuration, not only when BAGWIZ_WITH_SLAM is on.
namespace bagwiz::core::slam
{

// Write `points` as a binary-little-endian PLY mesh with no faces: each vertex
// carries `x y z` as float32. An `intensity` float32 property is emitted when
// `intensities` is non-empty AND exactly `points.size()` long; `uchar red green
// blue` properties are emitted when `colors` is non-empty AND exactly
// `points.size()` long. Internal colors are BGR triples; they are written as
// red/green/blue in the PLY header order. Mirrors `core::write_tum`'s void
// shape — the caller checks the stream state afterwards. Assumes a little-endian
// host (bagwiz targets x86).
void write_ply(
  std::ostream & os, std::span<const std::array<float, 3>> points,
  std::span<const float> intensities = {},
  std::span<const std::array<std::uint8_t, 3>> colors = {});

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__POINT_CLOUD_IO_HPP_
