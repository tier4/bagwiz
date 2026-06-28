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
#include <numeric>
#include <stdexcept>
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

namespace
{
// pi without relying on the POSIX M_PI macro under strict-ISO flags.
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegPerRad = 180.0 / kPi;
// Sentinel for "no point in this pixel". Must be larger than any real range.
constexpr float kNoPoint = 10000.0F;

// Project a world-frame point relative to `origin` into a dense range-image
// pixel. Out-of-FOV values are clamped to the nearest edge, matching upstream.
// Returns {-1,-1} only when the point coincides with the origin.
std::pair<std::int32_t, std::int32_t> project_to_pixel(
  const std::array<double, 3> & origin, const std::array<float, 3> & p, int rows, int cols,
  double vfov_deg, double hfov_deg, float * range_out)
{
  const double dx = static_cast<double>(p[0]) - origin[0];
  const double dy = static_cast<double>(p[1]) - origin[1];
  const double dz = static_cast<double>(p[2]) - origin[2];
  const double r = std::sqrt(dx * dx + dy * dy + dz * dz);
  if (range_out != nullptr) {
    *range_out = static_cast<float>(r);
  }
  if (r <= 0.0) {
    return {-1, -1};
  }

  const double az = std::atan2(dy, dx) * kDegPerRad;
  const double el = std::asin(std::max(-1.0, std::min(1.0, dz / r))) * kDegPerRad;

  const double row_f = rows * (1.0 - (el + vfov_deg / 2.0) / vfov_deg);
  const double col_f = cols * ((az + hfov_deg / 2.0) / hfov_deg);

  std::int32_t row = static_cast<std::int32_t>(std::round(row_f));
  std::int32_t col = static_cast<std::int32_t>(std::round(col_f));
  row = std::max(0, std::min(rows - 1, row));
  col = std::max(0, std::min(cols - 1, col));
  return {row, col};
}
}  // namespace

RemovertFilter::RemovertFilter(
  const RemovertConfig & config, const std::vector<std::array<float, 3>> & map_points)
: config_(config), map_points_(map_points)
{
}

void RemovertFilter::add_scan(
  const std::array<double, 3> & origin, const std::vector<std::array<float, 3>> & points)
{
  scans_.push_back({origin, points});
}

void RemovertFilter::add_scan(
  const std::array<double, 3> & origin, std::vector<std::array<float, 3>> && points)
{
  scans_.push_back({origin, std::move(points)});
}

std::vector<char> RemovertFilter::dynamic_mask_for_alpha(
  const std::vector<std::array<float, 3>> & map_points, double alpha,
  const std::vector<char> * candidate_mask) const
{
  const int rows = static_cast<int>(std::round(config_.vertical_fov_deg * alpha));
  const int cols = static_cast<int>(std::round(config_.horizontal_fov_deg * alpha));
  if (rows <= 0 || cols <= 0 || map_points.empty() || scans_.empty()) {
    return std::vector<char>(map_points.size(), 0);
  }

  constexpr std::size_t kMaxPixels = 50'000'000;
  const std::size_t pixel_count = static_cast<std::size_t>(rows) * cols;
  if (pixel_count > kMaxPixels) {
    throw std::invalid_argument(
      "Removert resolution would require " + std::to_string(pixel_count) +
      " range-image pixels (max " + std::to_string(kMaxPixels) + ")");
  }
  std::vector<char> dynamic(map_points.size(), 0);

  for (const auto & scan : scans_) {
    // Scan range image: closest observed range per pixel.
    std::vector<float> scan_range_image(pixel_count, kNoPoint);
    for (const auto & p : scan.points) {
      float r = 0.0F;
      const auto [row, col] = project_to_pixel(
        scan.origin, p, rows, cols, config_.vertical_fov_deg, config_.horizontal_fov_deg, &r);
      if (row < 0) {
        continue;
      }
      const std::size_t idx = static_cast<std::size_t>(row) * cols + col;
      if (r < scan_range_image[idx]) {
        scan_range_image[idx] = r;
      }
    }

    // Map range image: closest map range and the index of the map point.
    std::vector<float> map_range_image(pixel_count, kNoPoint);
    std::vector<std::int32_t> map_idx(pixel_count, -1);
    for (std::size_t i = 0; i < map_points.size(); ++i) {
      if (candidate_mask != nullptr && (*candidate_mask)[i] == 0) {
        continue;
      }
      float r = 0.0F;
      const auto [row, col] = project_to_pixel(
        scan.origin, map_points[i], rows, cols, config_.vertical_fov_deg,
        config_.horizontal_fov_deg, &r);
      if (row < 0) {
        continue;
      }
      const std::size_t idx = static_cast<std::size_t>(row) * cols + col;
      if (r < map_range_image[idx]) {
        map_range_image[idx] = r;
        map_idx[idx] = static_cast<std::int32_t>(i);
      }
    }

    // Per-pixel adaptive discrepancy rule from upstream removert.
    const float valid_max = static_cast<float>(config_.valid_diff_upper_bound);
    for (std::size_t px = 0; px < pixel_count; ++px) {
      const float scan_r = scan_range_image[px];
      const float map_r = map_range_image[px];
      const std::int32_t pt_idx = map_idx[px];
      if (scan_r >= kNoPoint || map_r >= kNoPoint || pt_idx < 0) {
        continue;
      }
      const float diff = std::abs(scan_r - map_r);
      const float threshold = static_cast<float>(config_.adaptive_coeff * scan_r);
      if (diff < valid_max && diff > threshold) {
        dynamic[static_cast<std::size_t>(pt_idx)] = 1;
      }
    }
  }

  return dynamic;
}

std::vector<char> RemovertFilter::filter()
{
  removed_count_ = 0;
  reverted_count_ = 0;

  if (map_points_.empty()) {
    return {};
  }

  // Sequential remove: each resolution operates on the map left by the previous.
  std::vector<std::array<float, 3>> current_map = map_points_;
  std::vector<std::size_t> current_to_original(map_points_.size());
  std::iota(current_to_original.begin(), current_to_original.end(), 0U);

  // Union of all points removed during the sequential chain.
  std::vector<char> ever_removed(map_points_.size(), 0);

  for (double alpha : config_.remove_resolutions) {
    const auto dynamic = dynamic_mask_for_alpha(current_map, alpha, nullptr);
    std::vector<std::array<float, 3>> next_map;
    std::vector<std::size_t> next_to_original;
    next_map.reserve(current_map.size());
    next_to_original.reserve(current_map.size());
    for (std::size_t i = 0; i < dynamic.size(); ++i) {
      if (dynamic[i] == 0) {
        next_map.push_back(current_map[i]);
        next_to_original.push_back(current_to_original[i]);
      } else {
        ever_removed[current_to_original[i]] = 1;
      }
    }
    current_map = std::move(next_map);
    current_to_original = std::move(next_to_original);
  }

  std::vector<char> keep(map_points_.size(), 1);
  for (std::size_t i = 0; i < ever_removed.size(); ++i) {
    if (ever_removed[i]) {
      keep[i] = 0;
    }
  }

  // Consensus revert: recover points that are not dynamic at any revert resolution.
  if (config_.enable_revert) {
    for (double alpha : config_.revert_resolutions) {
      std::vector<char> candidate(map_points_.size(), 0);
      for (std::size_t i = 0; i < keep.size(); ++i) {
        if (keep[i] == 0) {
          candidate[i] = 1;
        }
      }
      const auto dynamic = dynamic_mask_for_alpha(map_points_, alpha, &candidate);
      for (std::size_t i = 0; i < dynamic.size(); ++i) {
        if (keep[i] == 0 && dynamic[i] == 0) {
          keep[i] = 1;
          ++reverted_count_;
        }
      }
    }
  }

  removed_count_ = 0;
  for (char k : keep) {
    if (k == 0) {
      ++removed_count_;
    }
  }

  return keep;
}

}  // namespace bagwiz::core::slam
