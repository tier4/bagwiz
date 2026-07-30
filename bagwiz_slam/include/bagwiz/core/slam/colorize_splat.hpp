// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SLAM__COLORIZE_SPLAT_HPP_
#define BAGWIZ__CORE__SLAM__COLORIZE_SPLAT_HPP_

#include <array>

// Screen-space splat footprint for the SLAM map colorizer's depth buffer
// (`map slam --color`), extracted from the rasterizers so the projection
// geometry is unit testable on its own.
//
// The colorizer's occlusion buffer does not stamp a projected map point into
// its single pixel: the map is sparser than the pixel grid, so a lone pixel
// per point leaves holes a distant occluded point slips through. Each point is
// instead splatted over the footprint its LOCAL SURFACE covers on screen — a
// surfel: the disc of radius spacing/2 centered on the point, lying in the
// plane the point's normal defines.
//
// A disc projects to an ELLIPSE, not a circle, and the difference is the whole
// point of this file. A surface seen nearly edge-on — the road ahead of a
// vehicle-mounted camera, the facades running alongside it — projects to a
// sliver, and its samples pile up on screen along the tilt direction while
// staying spread out across it. Stamping each of those points with a full
// circle makes them occlude their own neighbors; stamping the projected disc
// covers exactly the direction where the samples are sparse and stops where
// they are dense. GLIM-free plain data throughout, like point_cloud_io.
namespace bagwiz::core::slam
{

// The screen-space ellipse a splat covers, as the symmetric 2x2 matrix C of
// the point set { q : q^T C^-1 q <= 1 } around the projected center, with q in
// pixels. Storing C (not its inverse) keeps the degenerate edge-on case — C
// singular, the ellipse collapsed to a segment — representable and division
// free; contains() tests the adjugate form instead of inverting.
//
// A zero matrix is the "center pixel only" footprint: half_extent_* are 0 and
// contains() accepts nothing, which is what both rasterizers want since they
// always write the center pixel unconditionally.
struct SplatFootprint
{
  double c_uu = 0.0;
  double c_uv = 0.0;
  double c_vv = 0.0;

  // Half-width and half-height of the ellipse's axis-aligned bounding box, in
  // pixels: the exact support of C along each axis (sqrt of the diagonal).
  // The rasterizers iterate this box and reject with contains().
  [[nodiscard]] double half_extent_u() const;
  [[nodiscard]] double half_extent_v() const;

  // True when the pixel offset (du, dv) from the splat center lies inside the
  // ellipse. Self-contained: callers need not also clip to the bounding box.
  //
  // The shape test is q^T adj(C) q <= det(C), the inverse-free equivalent of
  // q^T C^-1 q <= 1, which stays finite where the inverse would overflow. On
  // its own that test is unbounded along a singular C's collapsed direction
  // (det = 0 accepts the whole line, and an exactly edge-on surfel gets there
  // — as does FP32 cancellation in the CUDA mirror), so a length test
  // |q|^2 <= trace(C) runs alongside it. For a rank-1 C, trace(C) is exactly
  // the squared half-length of the segment the disc collapses to; for a
  // full-rank C it is >= the major semi-axis squared and so never rejects a
  // point the shape test admits.
  [[nodiscard]] bool contains(double du, double dv) const;
};

// Isotropic disc of radius `radius_px`, the footprint used where no surface
// orientation is available: map points whose neighborhood PCA produced no
// normal, and the per-image LiDAR scan splat (raw returns carry no normals).
// Non-positive radii yield the zero (center-pixel-only) footprint.
[[nodiscard]] SplatFootprint isotropic_splat_footprint(double radius_px);

// Footprint of a surfel: the disc of radius `radius_world` [m] centered at
// camera-frame `p_cam` in the plane with unit normal `n_cam` (camera frame;
// the sign is irrelevant, the footprint depends on n n^T).
//
// This is the EWA-splatting form specialized to a pinhole projection. With
// J the 2x3 Jacobian of (u, v) = (fx * x/z + cx, fy * y/z + cy) at p_cam and
// (I - n n^T) the projector onto the surfel's plane, the disc maps to the
// ellipse with C = radius_world^2 * J (I - n n^T) J^T — no tangent basis
// needed. Lens distortion is deliberately left out of J: the footprint is an
// occlusion heuristic a few pixels wide, and the circular radius it replaced
// ignored distortion too.
//
// A zero `n_cam` (the "no normal" sentinel from the geometry pre-pass) falls
// back to the isotropic disc of radius f_avg * radius_world / z with
// f_avg = (fx + fy) / 2 — bit-for-bit the footprint this file replaced, so a
// caller that supplies no normals rasterizes exactly as before.
//
// When the major semi-axis exceeds `max_axis_px` the whole ellipse is scaled
// down uniformly to that bound, preserving its aspect: the cap exists to stop
// an isolated point (whose spacing is huge) from blanketing the image, not to
// reshape the surfel. A non-positive `z` or `radius_world` yields the zero
// footprint.
[[nodiscard]] SplatFootprint surfel_splat_footprint(
  const std::array<double, 3> & p_cam, const std::array<double, 3> & n_cam, double radius_world,
  double fx, double fy, double max_axis_px);

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__COLORIZE_SPLAT_HPP_
