// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SLAM__CLOUD_FILTERS_HPP_
#define BAGWIZ__CORE__SLAM__CLOUD_FILTERS_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

// GLIM-free point-cloud filters. Kept in bagwiz_core (no GLIM dependency) so the
// logic is unit-testable on every distro without the SLAM build, and so the
// `slam` command's exported-map density is controlled by plain, deterministic
// code rather than by GLIM's internal parameters.
namespace bagwiz::core::slam
{

// Streaming voxel-grid downsampler. Points are bucketed into cubic voxels of side
// `resolution` [m]; each occupied voxel collapses to the centroid of the points
// that fell in it (and the mean intensity, when intensities are accumulated).
//
// This is how CloudMapper decouples the exported map's density from GLIM's
// internal sub-map density: every frame's globally-optimized points are streamed
// through one VoxelGrid, so the export resolution is independent of whatever
// sampling GLIM used for the optimization.
//
// Memory is bounded by the number of *occupied* voxels, not by the number of
// points added, so an arbitrarily large, heavily-overlapping input collapses to
// one entry per voxel. Output order is the order voxels were first seen, which is
// deterministic for a deterministic input stream (preserving the CPU
// reproducibility guarantee).
class VoxelGrid
{
public:
  // `resolution` is the voxel side length in meters; must be > 0 (a non-positive
  // value is clamped to a tiny positive epsilon to avoid division by zero).
  // `with_intensity` selects whether per-point intensities are accumulated and
  // returned; when false, intensities() is empty and the intensity argument to
  // add() is ignored.
  VoxelGrid(double resolution, bool with_intensity);

  // Add one point with no intensity (used when with_intensity == false).
  void add(float x, float y, float z);

  // Add one point with an intensity (accumulated only when with_intensity).
  void add(float x, float y, float z, float intensity);

  // Number of occupied voxels accumulated so far.
  [[nodiscard]] std::size_t size() const noexcept { return accum_.size(); }

  // Centroid of each occupied voxel, in first-seen order.
  [[nodiscard]] std::vector<std::array<float, 3>> points() const;

  // Mean intensity per occupied voxel, parallel to points(). Empty when the grid
  // was constructed with_intensity == false.
  [[nodiscard]] std::vector<float> intensities() const;

private:
  struct Key
  {
    std::int32_t x;
    std::int32_t y;
    std::int32_t z;
    bool operator==(const Key & other) const
    {
      return x == other.x && y == other.y && z == other.z;
    }
  };
  struct KeyHash
  {
    std::size_t operator()(const Key & key) const;
  };
  struct Accum
  {
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;
    double sum_intensity = 0.0;
    std::uint64_t count = 0;
  };

  void accumulate(float x, float y, float z, float intensity);

  double resolution_;
  bool with_intensity_;
  std::unordered_map<Key, std::size_t, KeyHash> index_;  // voxel -> slot in accum_
  std::vector<Accum> accum_;                             // first-seen order
};

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__CLOUD_FILTERS_HPP_
