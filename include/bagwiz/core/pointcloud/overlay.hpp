// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__POINTCLOUD__OVERLAY_HPP_
#define BAGWIZ__CORE__POINTCLOUD__OVERLAY_HPP_

#include "bagwiz/core/image/packed_raster.hpp"
#include "bagwiz/core/pointcloud/color_scheme.hpp"
#include "bagwiz/core/pointcloud/projector.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace bagwiz::core::pointcloud
{

// Draw `projected` points onto a copy of `src` using OpenCV, writing the
// result into `out`. `out` must have the same dimensions as `src`.
// Returns an empty string on success, otherwise an error message.
[[nodiscard]] std::string overlay_projected_points(
  const image::PackedRaster & src, const std::vector<ProjectedPoint> & projected,
  double property_min, double property_max, ColorScheme scheme, std::uint32_t point_size,
  float alpha, image::PackedRaster & out);

}  // namespace bagwiz::core::pointcloud

#endif  // BAGWIZ__CORE__POINTCLOUD__OVERLAY_HPP_
