// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/colorize_weight.hpp"

#include "bagwiz/core/image/gradient.hpp"

#include <algorithm>
#include <cmath>

namespace bagwiz::core::slam
{

double observation_distance_weight(float depth, double distance_ref)
{
  const double z = static_cast<double>(depth);
  const double ref_over_z = distance_ref / z;
  return std::clamp(ref_over_z * ref_over_z, 0.0, 1.0);
}

double observation_incidence_weight(
  const std::array<float, 3> & normal, const std::array<float, 3> & point,
  const std::array<double, 3> & cam_center)
{
  const auto & n = normal;
  const double nn = n[0] * n[0] + n[1] * n[1] + n[2] * n[2];
  if (nn > 0.0) {
    const auto & p = point;
    const double dx = static_cast<double>(p[0]) - cam_center[0];
    const double dy = static_cast<double>(p[1]) - cam_center[1];
    const double dz = static_cast<double>(p[2]) - cam_center[2];
    const double len = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (len > 0.0) {
      return std::abs(n[0] * dx + n[1] * dy + n[2] * dz) / (std::sqrt(nn) * len);
    }
  }
  return 1.0;
}

double observation_sharpness_weight(
  std::span<const std::byte> bgr, std::uint32_t width, std::uint32_t height, double u, double v,
  double sharpness_g0)
{
  const double g = image::sobel_gradient_magnitude_bilinear(bgr, width, height, u, v);
  return sharpness_g0 > 0.0 ? g / (g + sharpness_g0) : 1.0;
}

double observation_border_weight(
  double u, double v, std::uint32_t width, std::uint32_t height, double border_margin_px)
{
  const double edge = std::min(
    std::min(u, v),
    std::min(static_cast<double>(width - 1) - u, static_cast<double>(height - 1) - v));
  return border_margin_px > 0.0 ? std::clamp(edge / border_margin_px, 0.0, 1.0) : 1.0;
}

double compute_observation_weight(
  const VisiblePoint & vp, std::span<const std::array<float, 3>> points,
  std::span<const std::array<float, 3>> normals, const std::array<double, 3> & cam_center,
  std::span<const std::byte> bgr, std::uint32_t width, std::uint32_t height,
  const ObservationWeightParams & params)
{
  const double w_dist = observation_distance_weight(vp.depth, params.distance_ref);
  double w_inc = 1.0;
  if (vp.index < normals.size()) {
    w_inc = observation_incidence_weight(normals[vp.index], points[vp.index], cam_center);
  }
  const double w_sharp =
    observation_sharpness_weight(bgr, width, height, vp.u, vp.v, params.sharpness_g0);
  const double w_border =
    observation_border_weight(vp.u, vp.v, width, height, params.border_margin_px);
  return w_dist * w_inc * w_sharp * w_border;
}

}  // namespace bagwiz::core::slam
