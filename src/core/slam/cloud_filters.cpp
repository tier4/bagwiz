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
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <utility>
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

namespace
{
// 180/pi. A literal (not 180/M_PI) keeps this independent of whether <cmath>
// exposes the POSIX M_PI macro under the project's strict-ISO flags.
constexpr double kRadToDeg = 57.295779513082320876798154814105;
// Smallest spatial-hash cell we will divide by; guards a non-positive max_range
// (the CLI rejects those, but the class is reusable).
constexpr double kMinCellSize = 1e-3;

// A range image keyed by integer (azimuth, elevation) direction bin, holding the
// closest observed range per bin. Built fresh per scan from that scan's points.
struct AzElHash
{
  std::size_t operator()(const std::pair<std::int32_t, std::int32_t> & key) const
  {
    // Same Teschner-style mixing as the voxel hash; reinterpret to unsigned so
    // negative bins keep a clean bit pattern and the multiply wraps (defined).
    const auto a = static_cast<std::size_t>(static_cast<std::uint32_t>(key.first)) * kHashX;
    const auto b = static_cast<std::size_t>(static_cast<std::uint32_t>(key.second)) * kHashY;
    return a ^ b;
  }
};
using RangeImage = std::unordered_map<std::pair<std::int32_t, std::int32_t>, float, AzElHash>;

// Direction bin of a ray (dx,dy,dz) of length `range` from a scan origin.
// azimuth = atan2(y,x) in (-180,180], elevation = asin(z/range) in [-90,90].
std::pair<std::int32_t, std::int32_t> direction_bin(
  double dx, double dy, double dz, double range, double az_res, double el_res)
{
  // Clamp the sine to [-1, 1]: for a point on the polar axis, rounding can make
  // dz/range marginally exceed 1.0, and asin() of that is NaN (floor(NaN) -> UB).
  const double sin_el = std::max(-1.0, std::min(1.0, dz / range));
  const double az = std::atan2(dy, dx) * kRadToDeg;
  const double el = std::asin(sin_el) * kRadToDeg;
  return {
    static_cast<std::int32_t>(std::floor(az / az_res)),
    static_cast<std::int32_t>(std::floor(el / el_res))};
}
}  // namespace

std::size_t VisibilityFilter::CellHash::operator()(const CellKey & key) const
{
  return mix(key.x, kHashX) ^ mix(key.y, kHashY) ^ mix(key.z, kHashZ);
}

VisibilityFilter::CellKey VisibilityFilter::cell_of(double x, double y, double z) const
{
  return CellKey{
    static_cast<std::int32_t>(std::floor(x / cell_size_)),
    static_cast<std::int32_t>(std::floor(y / cell_size_)),
    static_cast<std::int32_t>(std::floor(z / cell_size_))};
}

VisibilityFilter::VisibilityFilter(
  const VisibilityFilterConfig & config, const std::vector<std::array<float, 3>> & map_points)
: config_(config),
  map_points_(map_points),
  observed_(map_points.size(), 0),
  seen_through_(map_points.size(), 0),
  cell_size_(config.max_range > kMinCellSize ? config.max_range : kMinCellSize)
{
  // Bucket every map point into a coarse grid (cell side = max_range) so each scan
  // only tests the points that could possibly fall within its range.
  for (std::uint32_t i = 0; i < map_points_.size(); ++i) {
    const auto & p = map_points_[i];
    grid_[cell_of(p[0], p[1], p[2])].push_back(i);
  }
}

void VisibilityFilter::add_scan(
  const std::array<double, 3> & origin, const std::vector<std::array<float, 3>> & points)
{
  if (map_points_.empty()) {
    return;
  }
  const double az_res = config_.azimuth_resolution_deg;
  const double el_res = config_.elevation_resolution_deg;
  const double min_r = config_.min_range;
  const double max_r = config_.max_range;
  const double margin = config_.range_margin;

  // 1) Build this scan's range image: closest observed range per direction bin.
  RangeImage range_image;
  range_image.reserve(points.size());
  for (const auto & p : points) {
    const double dx = static_cast<double>(p[0]) - origin[0];
    const double dy = static_cast<double>(p[1]) - origin[1];
    const double dz = static_cast<double>(p[2]) - origin[2];
    const double r = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (r < min_r || r > max_r) {
      continue;
    }
    const auto bin = direction_bin(dx, dy, dz, r, az_res, el_res);
    const auto it = range_image.find(bin);
    if (it == range_image.end()) {
      range_image.emplace(bin, static_cast<float>(r));
    } else if (static_cast<float>(r) < it->second) {
      it->second = static_cast<float>(r);
    }
  }
  if (range_image.empty()) {
    return;
  }

  // 2) Test only map points in the 3x3x3 cell neighborhood of the scan origin
  //    (cell side = max_range, so this covers every point within max_range).
  const CellKey base = cell_of(origin[0], origin[1], origin[2]);
  for (std::int32_t cx = base.x - 1; cx <= base.x + 1; ++cx) {
    for (std::int32_t cy = base.y - 1; cy <= base.y + 1; ++cy) {
      for (std::int32_t cz = base.z - 1; cz <= base.z + 1; ++cz) {
        const auto cell = grid_.find(CellKey{cx, cy, cz});
        if (cell == grid_.end()) {
          continue;
        }
        for (const std::uint32_t idx : cell->second) {
          const auto & m = map_points_[idx];
          const double dx = static_cast<double>(m[0]) - origin[0];
          const double dy = static_cast<double>(m[1]) - origin[1];
          const double dz = static_cast<double>(m[2]) - origin[2];
          const double r = std::sqrt(dx * dx + dy * dy + dz * dz);
          if (r < min_r || r > max_r) {
            continue;
          }
          const auto bin = direction_bin(dx, dy, dz, r, az_res, el_res);
          const auto seen = range_image.find(bin);
          if (seen == range_image.end()) {
            continue;  // scan never looked this way -> no evidence
          }
          const double observed_range = static_cast<double>(seen->second);
          if (observed_range > r + margin) {
            // Scan saw a surface farther along this ray: the map point was in
            // free space at scan time -> dynamic evidence.
            ++observed_[idx];
            ++seen_through_[idx];
          } else if (observed_range >= r - margin) {
            // A return near the map point's range: static support.
            ++observed_[idx];
          }
          // else: scan saw a closer surface (the point is occluded) -> no judgment.
        }
      }
    }
  }
}

std::vector<char> VisibilityFilter::keep_mask() const
{
  std::vector<char> keep(map_points_.size(), 1);
  const auto min_obs = static_cast<std::uint32_t>(std::max(config_.min_observations, 0));
  for (std::size_t i = 0; i < map_points_.size(); ++i) {
    if (
      observed_[i] >= min_obs &&
      static_cast<double>(seen_through_[i]) > config_.dynamic_ratio * observed_[i]) {
      keep[i] = 0;
    }
  }
  return keep;
}

std::size_t VisibilityFilter::removed_count() const
{
  const auto mask = keep_mask();
  std::size_t removed = 0;
  for (const char k : mask) {
    if (k == 0) {
      ++removed;
    }
  }
  return removed;
}

}  // namespace bagwiz::core::slam
