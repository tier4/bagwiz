// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SLAM__DYNAMIC_REMOVAL_HPP_
#define BAGWIZ__CORE__SLAM__DYNAMIC_REMOVAL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>

// GLIM-free and Eigen-free dynamic-point (moving-object ghost) classification,
// a native reimplementation of DUFOMap's void-region method (Duberg et al.,
// RA-L 2024): every scan's rays mark the voxels they traverse as seen-free, and
// a point falling in a voxel that was ever seen free (a "void") must have been
// measured on something that later moved away (or arrived later), so it is
// dynamic. Kept in the plain bagwiz_slam library (like cloud_filters.hpp) so it
// builds and unit-tests in every configuration; the caller (CloudMapper)
// transforms scans to world-frame xyz, mirroring cloud_voxelize_gpu.hpp's
// convention.
namespace bagwiz::core::slam
{

// Tuning for the void-region classification. Defaults follow the DUFOMap
// paper's outdoor settings.
struct VoidRegionConfig
{
  // Side length [m] of the cubic free-space voxels. Independent of the export
  // resolution: coarser voxels cost less memory and absorb more pose noise but
  // blur the free/occupied boundary (ghosts closer than one voxel to a static
  // surface survive). Must be > 0 (clamped to a tiny epsilon otherwise).
  double voxel_size = 0.2;

  // d_s [m]: each ray stops this far short of its hit to keep sensor range
  // noise from marking the hit surface's neighborhood as free. The hit point's
  // own voxel is never marked free by its own ray regardless.
  double sensor_offset = 0.15;

  // d_p [voxels]: Chebyshev radius of the void erosion. A voxel is void only
  // when it AND every voxel within this radius were seen free, so a pose error
  // of up to roughly d_p * voxel_size cannot delete static points. 0 disables
  // the erosion (void == seen-free). Higher = more conservative removal.
  int neighborhood = 1;

  // Upper bound [m] on the traversed ray length; the free-space marking of a
  // longer ray is truncated there (the caller passes its range crop, so rays
  // never carve free space beyond the configured sensing range).
  double max_ray_length = 100.0;
};

// Void-region classifier over world-frame points. Three-phase use:
//   1. integrate() every scan (thread-safe; calls may run concurrently),
//   2. finalize() exactly once after all integration is done,
//   3. classify() each scan (const; calls may run concurrently).
// The voxel state is a pure OR over all rays — idempotent, commutative and
// monotone — so the classification is byte-identical for any thread count or
// integration order (unlike a probabilistic occupancy map, nothing decays).
// Memory is bounded by the traversed space: ~2 bits per touched voxel, held in
// block-sparse bitsets.
class VoidRegionClassifier
{
public:
  explicit VoidRegionClassifier(const VoidRegionConfig & config);
  ~VoidRegionClassifier();

  VoidRegionClassifier(const VoidRegionClassifier &) = delete;
  VoidRegionClassifier & operator=(const VoidRegionClassifier &) = delete;

  // Phase 1 — mark seen-free voxels: for each point, walk the voxels on the
  // segment from `sensor_origin` toward the point (Amanatides-Woo DDA),
  // stopping sensor_offset short of the hit and at max_ray_length, and never
  // marking the hit point's own voxel. Non-finite points and points at the
  // origin are skipped. Thread-safe.
  void integrate(
    std::span<const std::array<float, 3>> world_points,
    const std::array<double, 3> & sensor_origin);

  // Phase 2 — build the void mask by eroding the seen-free set by
  // `neighborhood` (Chebyshev). Must be called exactly once, after every
  // integrate() call has returned; `num_threads` bounds the erosion workers
  // (values < 1 are treated as 1) and does not affect the result.
  void finalize(int num_threads);

  // Phase 3 — keep[i] = 0 iff world_points[i] lies in a void voxel, else 1.
  // Returns the number of dynamic (keep == 0) slots written. keep.size() must
  // be >= world_points.size(). Requires finalize(); const and lock-free, so
  // concurrent calls are safe.
  std::size_t classify(
    std::span<const std::array<float, 3>> world_points, std::span<std::uint8_t> keep) const;

  // True when the voxel containing (x, y, z) was traversed by any ray. Valid
  // once the integrate() calls have completed (finalize() not required).
  [[nodiscard]] bool seen_free(float x, float y, float z) const;

  // True when the voxel containing (x, y, z) is void (seen-free after the
  // erosion). Requires finalize().
  [[nodiscard]] bool is_void(float x, float y, float z) const;

  // Number of voxels marked seen-free so far (diagnostic; single-threaded use).
  [[nodiscard]] std::size_t seen_free_voxel_count() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__DYNAMIC_REMOVAL_HPP_
