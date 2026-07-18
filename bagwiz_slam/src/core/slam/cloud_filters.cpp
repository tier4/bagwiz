// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/slam/cloud_filters.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace bagwiz::core::slam
{
namespace
{
// Smallest voxel side we will divide by; guards against a non-positive
// resolution slipping through (the CLI already rejects those, but the class is
// reusable). 1 mm is far below any meaningful map resolution.
constexpr double kMinResolution = 1e-3;

// Mixed integer hash for a 3D voxel index (the classic Teschner et al. spatial
// hash constants). Unsigned so the multiply wraps (defined) rather than risking
// signed-integer overflow (UB). Good enough spread for unordered_map bucketing.
constexpr std::size_t kHashX = 73856093U;
constexpr std::size_t kHashY = 19349663U;
constexpr std::size_t kHashZ = 83492791U;

// One axis's contribution to the hash. A voxel index is reinterpreted (not
// value-converted) to unsigned so negative indices keep a clean bit pattern.
std::size_t mix(std::int32_t index, std::size_t multiplier)
{
  return static_cast<std::size_t>(static_cast<std::uint32_t>(index)) * multiplier;
}
}  // namespace

std::size_t VoxelGrid::KeyHash::operator()(const Key & key) const
{
  return mix(key.x, kHashX) ^ mix(key.y, kHashY) ^ mix(key.z, kHashZ);
}

VoxelGrid::VoxelGrid(double resolution, bool with_intensity)
: resolution_(resolution > kMinResolution ? resolution : kMinResolution),
  with_intensity_(with_intensity)
{
}

void VoxelGrid::add(float x, float y, float z)
{
  accumulate(x, y, z, 0.0F);
}

void VoxelGrid::add(float x, float y, float z, float intensity)
{
  accumulate(x, y, z, intensity);
}

void VoxelGrid::accumulate(float x, float y, float z, float intensity)
{
  // floor (not truncation) so the grid is continuous across zero — otherwise the
  // voxels straddling an axis would be twice as wide.
  const Key key{
    static_cast<std::int32_t>(std::floor(static_cast<double>(x) / resolution_)),
    static_cast<std::int32_t>(std::floor(static_cast<double>(y) / resolution_)),
    static_cast<std::int32_t>(std::floor(static_cast<double>(z) / resolution_))};

  const auto found = index_.find(key);
  Accum * cell = nullptr;
  if (found == index_.end()) {
    index_.emplace(key, accum_.size());
    keys_.push_back(key);  // parallel to accum_, in first-seen order (for merge_from)
    accum_.emplace_back();
    cell = &accum_.back();
  } else {
    cell = &accum_[found->second];
  }

  cell->sum_x += static_cast<double>(x);
  cell->sum_y += static_cast<double>(y);
  cell->sum_z += static_cast<double>(z);
  if (with_intensity_) {
    cell->sum_intensity += static_cast<double>(intensity);
  }
  ++cell->count;
}

std::vector<std::array<float, 3>> VoxelGrid::points() const
{
  std::vector<std::array<float, 3>> result;
  result.reserve(accum_.size());
  for (const auto & cell : accum_) {
    const auto n = static_cast<double>(cell.count);
    result.push_back(
      {static_cast<float>(cell.sum_x / n), static_cast<float>(cell.sum_y / n),
       static_cast<float>(cell.sum_z / n)});
  }
  return result;
}

std::vector<float> VoxelGrid::intensities() const
{
  std::vector<float> result;
  if (!with_intensity_) {
    return result;
  }
  result.reserve(accum_.size());
  for (const auto & cell : accum_) {
    result.push_back(static_cast<float>(cell.sum_intensity / static_cast<double>(cell.count)));
  }
  return result;
}

void VoxelGrid::merge_from(const VoxelGrid & other)
{
  // Both grids must agree on resolution + intensity mode, else merged bins and
  // intensity means would be silently wrong; the sole caller (fill_map_parallel)
  // builds every per-thread grid identically. A debug-only guard, zero cost in
  // release, matching how the codebase asserts internal-helper preconditions.
  assert(resolution_ == other.resolution_ && with_intensity_ == other.with_intensity_);
  // Iterate other in first-seen (accum_) order so the merge is deterministic;
  // its unordered_map index_ would iterate in an unspecified order.
  for (std::size_t i = 0; i < other.accum_.size(); ++i) {
    const Key & key = other.keys_[i];
    const Accum & src = other.accum_[i];
    const auto found = index_.find(key);
    if (found == index_.end()) {
      index_.emplace(key, accum_.size());
      keys_.push_back(key);
      accum_.push_back(src);  // copy the other grid's running sums + count verbatim
    } else {
      Accum & dst = accum_[found->second];
      dst.sum_x += src.sum_x;
      dst.sum_y += src.sum_y;
      dst.sum_z += src.sum_z;
      dst.sum_intensity += src.sum_intensity;
      dst.count += src.count;
    }
  }
}

}  // namespace bagwiz::core::slam
