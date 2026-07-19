// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SLAM__PROPAGATION_RADIUS_HPP_
#define BAGWIZ__CORE__SLAM__PROPAGATION_RADIUS_HPP_

#include <optional>
#include <span>

// The automatic radius for filling the map points no camera observed with the
// color of their nearest observed neighbor (`map slam --cam`, see
// pointcloud/color_propagation.hpp). The radius follows the data — 4x the
// median local point spacing, clamped to [0.05, 5] m — so it tracks the map
// density instead of an absolute guess. GLIM-free plain data throughout, like
// point_cloud_io.
namespace bagwiz::core::slam
{

// Compute the propagation radius from the per-point local spacings (see
// ColorizeGeometry::spacings). Degenerate inputs — empty spacings or a
// non-finite / non-positive median — yield std::nullopt; the caller then
// skips the propagation.
[[nodiscard]] std::optional<double> propagation_radius_from_spacings(
  std::span<const float> spacings);

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__PROPAGATION_RADIUS_HPP_
