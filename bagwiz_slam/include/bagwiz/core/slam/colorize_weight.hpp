// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SLAM__COLORIZE_WEIGHT_HPP_
#define BAGWIZ__CORE__SLAM__COLORIZE_WEIGHT_HPP_

#include "bagwiz/core/slam/colorize_rasterizer.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

// Per-observation weight math for the SLAM map colorizer (`map slam --cam`),
// extracted from MapColorizer's weight-and-sample pass so each term is unit
// testable in isolation. Every visible point's observation is weighted by how
// trustworthy it is — depth distance, surface incidence, image sharpness, and
// image border — and the product decides both whether the observation is kept
// (MapColorizerConfig::weight_min) and how strongly it counts in the point's
// reservoir. GLIM-free plain data throughout, like point_cloud_io.
namespace bagwiz::core::slam
{

// The three MapColorizerConfig knobs the observation weight reads, as plain
// data so the terms below stay free of the colorizer's configuration type.
struct ObservationWeightParams
{
  double distance_ref;      // depth [m] at which the distance weight saturates to 1
  double sharpness_g0;      // sharpness half-saturation; <= 0 disables the term
  double border_margin_px;  // border falloff width [px]; <= 0 disables the term
};

// w_dist = clamp((distance_ref / depth)^2, 0, 1): observations nearer than
// the reference depth are fully trusted, farther ones fall off with the
// inverse square of depth.
[[nodiscard]] double observation_distance_weight(float depth, double distance_ref);

// w_inc = |n . (p - c)| / (|n| * |p - c|): the cosine between the surface
// normal and the view direction, so surfaces seen at grazing angles count
// less. 1.0 when either vector degenerates (a zero normal carries no
// incidence information; a zero-length view direction has no direction).
[[nodiscard]] double observation_incidence_weight(
  const std::array<float, 3> & normal, const std::array<float, 3> & point,
  const std::array<double, 3> & cam_center);

// w_sharp = g / (g + sharpness_g0) where g is the bilinear Sobel gradient
// magnitude (|gx| + |gy|) of the image at (u, v): blurred or featureless
// pixels count less. 1.0 when sharpness_g0 <= 0.
[[nodiscard]] double observation_sharpness_weight(
  std::span<const std::byte> bgr, std::uint32_t width, std::uint32_t height, double u, double v,
  double sharpness_g0);

// w_border = clamp(edge / border_margin_px, 0, 1) where edge is the distance
// [px] from (u, v) to the nearest image border: pixels near the border count
// less (lens models are least reliable there). 1.0 when border_margin_px <= 0.
[[nodiscard]] double observation_border_weight(
  double u, double v, std::uint32_t width, std::uint32_t height, double border_margin_px);

// The full weight product w_dist * w_inc * w_sharp * w_border for one visible
// point. `normals` is the geometry pre-pass's per-point normals, parallel to
// `points` (empty when no geometry was built — the incidence term then
// defaults to 1); an index with no normal entry also yields incidence 1.
[[nodiscard]] double compute_observation_weight(
  const VisiblePoint & vp, std::span<const std::array<float, 3>> points,
  std::span<const std::array<float, 3>> normals, const std::array<double, 3> & cam_center,
  std::span<const std::byte> bgr, std::uint32_t width, std::uint32_t height,
  const ObservationWeightParams & params);

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__COLORIZE_WEIGHT_HPP_
