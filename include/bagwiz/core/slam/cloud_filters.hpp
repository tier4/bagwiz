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

// Tunables for RemovertFilter. Distances are meters, angles degrees. Defaults
// mirror the upstream irapkaist/removert KITTI example.
struct RemovertConfig
{
  // Vertical and horizontal field of view used to build the range images.
  // Vertical FOV is total (e.g., 50 means +/-25 degrees around the horizon).
  double vertical_fov_deg = 50.0;
  double horizontal_fov_deg = 360.0;

  // Resolution magnifiers (pixels per degree) for the map-side remove pass.
  // Processed in order; each resolution operates on the map left by the previous
  // one, exactly as upstream does.
  std::vector<double> remove_resolutions = {2.5, 2.0, 1.5};

  // Resolution magnifiers for the consensus revert pass. A removed point is
  // recovered if it is *not* classified as dynamic at any of these resolutions.
  std::vector<double> revert_resolutions = {1.0, 0.9, 0.8, 0.7};

  // A map point is marked dynamic when abs(scan_range - map_range) is larger
  // than this fraction of the scan range. Upstream hard-codes 0.05.
  double adaptive_coeff = 0.05;

  // Pixels whose scan/map range difference exceeds this value are treated as
  // no-point pixels and ignored. Upstream hard-codes 200 meters.
  double valid_diff_upper_bound = 200.0;

  // Enable the multi-resolution consensus revert pass.
  bool enable_revert = true;
};

// Original Removert-style dynamic-point removal. The filter is constructed with
// the merged map points, every optimized scan's viewpoint is fed via add_scan(),
// and filter() runs the sequential remove + consensus revert pipeline.
//
// The implementation follows the upstream irapkaist/removert algorithm:
// FOV-based dense range images, closest-return-per-pixel, and a per-pixel
// adaptive discrepancy rule. It is purely geometric, GLIM-free, and
// deterministic for a deterministic scan stream.
class RemovertFilter
{
public:
  // `map_points` is the accumulated map to filter (world-frame xyz), copied in.
  RemovertFilter(
    const RemovertConfig & config, const std::vector<std::array<float, 3>> & map_points);

  // Accumulate one scan's viewpoint. `origin` is the scan's sensor position in
  // the world; `points` are the world-frame points it observed.
  void add_scan(
    const std::array<double, 3> & origin, const std::vector<std::array<float, 3>> & points);

  // Move overload: avoids copying the scan points when the caller no longer needs
  // them. `points` is left in a moved-from state.
  void add_scan(const std::array<double, 3> & origin, std::vector<std::array<float, 3>> && points);

  // Run the remove + consensus revert pipeline on the accumulated scans and
  // return a per-map-point keep mask: 1 = static (keep), 0 = dynamic (drop).
  // Parallel to the constructor's map_points.
  [[nodiscard]] std::vector<char> filter();

  // Number of map points flagged as dynamic after the last filter() call.
  [[nodiscard]] std::size_t removed_count() const noexcept { return removed_count_; }

  // Number of points recovered by the consensus revert pass after the last
  // filter() call.
  [[nodiscard]] std::size_t reverted_count() const noexcept { return reverted_count_; }

private:
  struct ScanView
  {
    std::array<double, 3> origin;
    std::vector<std::array<float, 3>> points;
  };

  [[nodiscard]] std::vector<char> dynamic_mask_for_alpha(
    const std::vector<std::array<float, 3>> & map_points, double alpha,
    const std::vector<char> * candidate_mask) const;

  RemovertConfig config_;
  std::vector<std::array<float, 3>> map_points_;
  std::vector<ScanView> scans_;
  std::size_t removed_count_ = 0;
  std::size_t reverted_count_ = 0;
};

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__CLOUD_FILTERS_HPP_
