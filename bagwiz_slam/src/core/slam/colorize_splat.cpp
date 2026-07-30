// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/colorize_splat.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace bagwiz::core::slam
{

namespace
{

// Scales the whole ellipse down so its major semi-axis meets `max_axis_px`,
// leaving a footprint already within the bound untouched. Scaling C uniformly
// (rather than capping each axis) keeps the surfel's aspect: the cap exists to
// stop an isolated point's huge spacing from blanketing the image, not to
// undo the foreshortening the ellipse encodes.
SplatFootprint clamp_major_axis(const SplatFootprint & f, double max_axis_px)
{
  if (!(max_axis_px > 0.0)) {
    return SplatFootprint{};
  }
  // The major semi-axis squared is C's larger eigenvalue,
  // mean + sqrt(spread_sq). Deciding whether it exceeds the cap only needs the
  // squared comparison, so the square root stays off the common path — this
  // runs once per projected point per image, while the cap itself bites only
  // for the isolated points it exists to bound.
  const double mean = 0.5 * (f.c_uu + f.c_vv);
  const double half_diff = 0.5 * (f.c_uu - f.c_vv);
  const double spread_sq = half_diff * half_diff + f.c_uv * f.c_uv;
  const double max_sq = max_axis_px * max_axis_px;
  const double headroom = max_sq - mean;
  if (headroom >= 0.0 && spread_sq <= headroom * headroom) {
    return f;
  }
  // Reaching here means mean + sqrt(spread_sq) > max_sq > 0, so the divisor is
  // strictly positive.
  const double shrink = max_sq / (mean + std::sqrt(spread_sq));
  return SplatFootprint{f.c_uu * shrink, f.c_uv * shrink, f.c_vv * shrink};
}

}  // namespace

double SplatFootprint::half_extent_u() const
{
  return std::sqrt(std::max(0.0, c_uu));
}

double SplatFootprint::half_extent_v() const
{
  return std::sqrt(std::max(0.0, c_vv));
}

bool SplatFootprint::contains(double du, double dv) const
{
  const double du_sq = du * du;
  const double dv_sq = dv * dv;
  // Length test first: bounds the collapsed direction of a singular C, and is
  // implied by the shape test whenever C has full rank (see the header).
  if (du_sq + dv_sq > c_uu + c_vv) {
    return false;
  }
  // Shape test: q^T adj(C) q <= det(C), the inverse-free q^T C^-1 q <= 1.
  const double quadratic = c_vv * du_sq - 2.0 * c_uv * du * dv + c_uu * dv_sq;
  return quadratic <= c_uu * c_vv - c_uv * c_uv;
}

SplatFootprint isotropic_splat_footprint(double radius_px)
{
  if (!(radius_px > 0.0)) {
    return SplatFootprint{};
  }
  return SplatFootprint{radius_px * radius_px, 0.0, radius_px * radius_px};
}

SplatFootprint surfel_splat_footprint(
  const std::array<double, 3> & p_cam, const std::array<double, 3> & n_cam, double radius_world,
  double fx, double fy, double max_axis_px)
{
  const double z = p_cam[2];
  if (!(z > 0.0) || !(radius_world > 0.0)) {
    return SplatFootprint{};
  }

  const double n_norm_sq = n_cam[0] * n_cam[0] + n_cam[1] * n_cam[1] + n_cam[2] * n_cam[2];
  if (!(n_norm_sq > 0.0)) {
    // The geometry pre-pass's "no normal" sentinel: with no orientation to
    // foreshorten by, fall back to the fronto-parallel disc the rasterizers
    // used before, sized by the average focal length.
    return clamp_major_axis(
      isotropic_splat_footprint(0.5 * (fx + fy) * radius_world / z), max_axis_px);
  }

  // C = radius_world^2 * J (I - n n^T) J^T with J the projection Jacobian at
  // p_cam. Factoring J = K / z with K = [[fx, 0, -fx*a], [0, fy, -fy*b]] and
  // a = x/z, b = y/z leaves C = scale^2 * (K K^T - (K n)(K n)^T), which needs
  // no tangent basis for the surfel's plane. The (1 + a^2) / (1 + b^2)
  // diagonal terms are the perspective stretch away from the optical axis;
  // the (K n) rank-1 subtraction is the foreshortening.
  const double a = p_cam[0] / z;
  const double b = p_cam[1] / z;
  const double scale_sq = (radius_world / z) * (radius_world / z);

  // K n, with the fx / fy factors deferred to the products below so they
  // pair with the matching K K^T terms.
  const double kn_u = n_cam[0] - a * n_cam[2];
  const double kn_v = n_cam[1] - b * n_cam[2];

  return clamp_major_axis(
    SplatFootprint{
      scale_sq * fx * fx * (1.0 + a * a - kn_u * kn_u), scale_sq * fx * fy * (a * b - kn_u * kn_v),
      scale_sq * fy * fy * (1.0 + b * b - kn_v * kn_v)},
    max_axis_px);
}

}  // namespace bagwiz::core::slam
