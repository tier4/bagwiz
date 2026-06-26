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

// Tunables for VisibilityFilter (Removert-style dynamic-point removal). Distances
// are meters, angles degrees. Defaults are deliberately conservative: a map point
// is dropped only when a clear fraction of the scans that look along its line of
// sight see *past* it to a farther surface, which is the signature of a moving
// object's ghost (the space it occupied in one scan is observed as free in
// others). Static structure is observed at its own range and kept.
struct VisibilityFilterConfig
{
  // Range-image bin sizes. A scan's points are bucketed by their (azimuth,
  // elevation) direction *from that scan's origin*; a map point is compared
  // against the closest scan return that fell in the same bin. Too fine and a
  // background ray lands in an empty bin (missed evidence); too coarse and a
  // foreground edge hides the background behind it. ~0.5°/1.0° suits common
  // spinning LiDARs (dense in azimuth, sparser in elevation).
  double azimuth_resolution_deg = 0.5;
  double elevation_resolution_deg = 1.0;

  // A map point counts as "seen through" by a scan when that scan observed a
  // surface at least this much farther along the same direction (and nothing
  // near the point). Also the tolerance band for calling a return "at" the point
  // (static support). Absorbs range noise and pose error.
  double range_margin = 0.5;

  // A scan only constrains a map point whose range from the scan origin is within
  // [min_range, max_range]. min_range drops ego/near-body returns; max_range
  // bounds the per-scan candidate search and ignores far, sparse, unreliable
  // returns.
  double min_range = 1.0;
  double max_range = 60.0;

  // A map point must be observed (in-range, in an observed direction bin) by at
  // least this many scans before it can be judged, so a single noisy look cannot
  // delete a point.
  int min_observations = 2;

  // Remove a map point when seen_through / observed exceeds this ratio. Higher =
  // more conservative (keeps more). 0.3 removes points the majority-ish of looks
  // see past while leaving consistently-observed structure intact.
  double dynamic_ratio = 0.3;
};

// Removert-style visibility filter that flags dynamic ("see-through") points in
// an accumulated map. GLIM-free and streaming so it is unit-testable without the
// SLAM build, mirroring VoxelGrid: construct it with the merged map points, feed
// every scan's viewpoint (sensor origin + the world-frame points it observed)
// via add_scan(), then read keep_mask().
//
// The test is purely geometric and needs no sensor orientation: for a scan with
// origin O, each direction is (p - O) in world axes, so a map point's
// direction-and-range from O is compared against the closest scan return in the
// same direction bin. If the scan saw a surface farther than the map point
// (margin), the line of sight to the point was clear at that time -> evidence the
// point is dynamic. Evidence is accumulated across all scans; a point is dropped
// only when the dynamic fraction clears the configured ratio (and it was seen
// enough times). Static surfaces are observed at their own range and survive.
//
// Cost is bounded by a coarse spatial hash (cell = max_range): each scan tests
// only map points in the 3x3x3 neighborhood of its origin cell, so the work is
// proportional to locally-visible points rather than the whole map per scan.
// Deterministic for a deterministic scan stream (map order preserved), keeping
// the slam command's CPU-reproducibility guarantee.
class VisibilityFilter
{
public:
  // `map_points` is the accumulated map to filter (world-frame xyz), copied in.
  VisibilityFilter(
    const VisibilityFilterConfig & config, const std::vector<std::array<float, 3>> & map_points);

  // Accumulate one scan's visibility evidence. `origin` is the scan's sensor
  // position in the world; `points` are the world-frame points it observed.
  void add_scan(
    const std::array<double, 3> & origin, const std::vector<std::array<float, 3>> & points);

  // Per-map-point keep flag, parallel to the constructor's map_points: 1 = static
  // (keep), 0 = dynamic (drop). Computed from the evidence gathered so far.
  [[nodiscard]] std::vector<char> keep_mask() const;

  // Number of map points keep_mask() currently flags as dynamic.
  [[nodiscard]] std::size_t removed_count() const;

private:
  struct CellKey
  {
    std::int32_t x;
    std::int32_t y;
    std::int32_t z;
    bool operator==(const CellKey & other) const
    {
      return x == other.x && y == other.y && z == other.z;
    }
  };
  struct CellHash
  {
    std::size_t operator()(const CellKey & key) const;
  };

  CellKey cell_of(double x, double y, double z) const;

  VisibilityFilterConfig config_;
  std::vector<std::array<float, 3>> map_points_;
  std::vector<std::uint32_t> observed_;      // per map point: in-range observations
  std::vector<std::uint32_t> seen_through_;  // per map point: see-through looks
  // Spatial hash over map points (cell side = max_range) for per-scan candidate
  // lookup. Built once at construction.
  std::unordered_map<CellKey, std::vector<std::uint32_t>, CellHash> grid_;
  double cell_size_;
};

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__CLOUD_FILTERS_HPP_
