// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SLAM__GNSS_ALIGNMENT_HPP_
#define BAGWIZ__CORE__SLAM__GNSS_ALIGNMENT_HPP_

#include <array>
#include <vector>

// Horizontal (2-D) rigid alignment between SLAM-estimated submap origins and
// their associated GNSS positions, ported from glim_ext's gnss_global. The
// best-fit yaw + x-y translation (z carried but not rotated) is the rigid
// transform that maps the GNSS frame onto the SLAM world frame; each GNSS
// position mapped through it becomes the target of that submap's translation
// prior.
//
// Kept GLIM-free and Eigen-free (closed-form 2-D Procrustes, see the .cpp) so it
// lives in the plain bagwiz_slam library, which builds in every configuration,
// and is unit-tested without the GLIM stack — this is the
// numerically interesting half of the GNSS constraint, so it is the half worth
// testing in isolation.
namespace bagwiz::core::slam
{

// Given parallel SLAM submap origins `est` and GNSS positions `gnss` (each a
// metric {x, y, z}), estimate the world<-GNSS transform from their horizontal
// components and return each GNSS position mapped into the world frame — i.e.
// the translation-prior target for the corresponding submap.
//
// Returns an empty vector when the inputs are empty or of unequal length. With a
// single pair (or near-degenerate horizontal spread) the rotation is left at
// identity and only the translation is fitted, matching glim_ext's behavior.
[[nodiscard]] std::vector<std::array<double, 3>> align_gnss_to_world(
  const std::vector<std::array<double, 3>> & est, const std::vector<std::array<double, 3>> & gnss);

// Result of gnss_targets_with_offset: the per-submap prior targets plus the planar
// rotation that carries an ENU covariance into the SLAM world frame.
struct GnssOffsetTargets
{
  // Prior-target position per submap (SLAM world frame), parallel to the inputs.
  // Empty when the inputs are empty/mismatched or the alignment is degenerate.
  std::vector<std::array<double, 3>> targets;

  // The world<-GNSS(ENU) rotation fitted during alignment, as the 2x2 map
  // M = [[cos, sin], [-sin, cos]] acting on horizontal {East, North} vectors: a
  // horizontal ENU covariance Sigma rotates into the world frame as M * Sigma * M^T
  // (see gnss_world_prior_covariance). Identity (cos=1, sin=0) when the alignment
  // is degenerate or the inputs are empty.
  double world_from_enu_cos = 1.0;
  double world_from_enu_sin = 0.0;
};

// Lever-arm-aware variant of align_gnss_to_world: the GNSS antenna sits at a
// rigid offset from the submap-origin sensor, so the value a `NavSatFix` reports
// is the *antenna* position, not the submap origin. `offsets_world[i]` is that
// per-submap antenna offset already rotated into the world frame
// (R_world_origin · lever_arm); passing it lets the rigid world<-GNSS fit run
// antenna-to-antenna (uncontaminated by the heading-dependent offset), and the
// per-submap prior target is then mapped back onto the submap origin by removing
// the same offset. The fitted ENU->world rotation is returned too so each fix's
// ENU covariance can be expressed in the world frame for prior weighting.
//
// Equivalent to align_gnss_to_world(origins, gnss) (and an identity rotation) when
// every offset is zero. Returns empty targets when the three inputs are empty or
// of unequal length.
[[nodiscard]] GnssOffsetTargets gnss_targets_with_offset(
  const std::vector<std::array<double, 3>> & origins,
  const std::vector<std::array<double, 3>> & offsets_world,
  const std::vector<std::array<double, 3>> & gnss);

// Build the 3x3 world-frame position covariance (m^2, row-major) for one GNSS
// translation prior. `cov_enu_horizontal` is the fix's horizontal ENU covariance
// as {c_ee, c_en, c_ne, c_nn} (row-major 2x2); it is rotated into the world frame
// by M = [[cos, sin], [-sin, cos]] (Sigma_w = M * Sigma * M^T, cos/sin from
// gnss_targets_with_offset), scaled by `inflation`^2 (GNSS formal covariance is
// optimistic and consecutive fixes are time-correlated), then floored by adding
// `floor_sigma`^2 to each horizontal diagonal (an isotropic, PSD-preserving
// minimum-noise floor). The vertical entry is set to `z_variance` directly (pass a
// large value to leave height effectively unconstrained, mirroring the
// fixed-precision path's z = 0 information). Off-diagonal z terms are zero.
[[nodiscard]] std::array<double, 9> gnss_world_prior_covariance(
  const std::array<double, 4> & cov_enu_horizontal, double world_from_enu_cos,
  double world_from_enu_sin, double floor_sigma, double inflation, double z_variance);

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__GNSS_ALIGNMENT_HPP_
