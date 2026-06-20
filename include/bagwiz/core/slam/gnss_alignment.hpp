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
// lives in bagwiz_core and is unit-tested without the GLIM stack — this is the
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

// Lever-arm-aware variant of align_gnss_to_world: the GNSS antenna sits at a
// rigid offset from the submap-origin sensor, so the value a `NavSatFix` reports
// is the *antenna* position, not the submap origin. `offsets_world[i]` is that
// per-submap antenna offset already rotated into the world frame
// (R_world_origin · lever_arm); passing it lets the rigid world<-GNSS fit run
// antenna-to-antenna (uncontaminated by the heading-dependent offset), and the
// per-submap prior target is then mapped back onto the submap origin by removing
// the same offset.
//
// Equivalent to align_gnss_to_world(origins, gnss) when every offset is zero, so
// a missing/zero lever-arm reproduces the no-correction behavior exactly. Returns
// an empty vector when the three inputs are empty or of unequal length.
[[nodiscard]] std::vector<std::array<double, 3>> gnss_targets_with_offset(
  const std::vector<std::array<double, 3>> & origins,
  const std::vector<std::array<double, 3>> & offsets_world,
  const std::vector<std::array<double, 3>> & gnss);

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__GNSS_ALIGNMENT_HPP_
