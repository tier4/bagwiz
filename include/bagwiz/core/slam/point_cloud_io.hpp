// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__SLAM__POINT_CLOUD_IO_HPP_
#define BAGWIZ__CORE__SLAM__POINT_CLOUD_IO_HPP_

#include <array>
#include <cstdint>
#include <istream>
#include <ostream>
#include <span>
#include <string>
#include <vector>

// GLIM-free point-cloud I/O used by the `map` commands. Kept free of GLIM /
// Eigen / gtsam_points types so the reader/writer (and their tests) build in
// every configuration, not only when BAGWIZ_WITH_SLAM is on.
namespace bagwiz::core::slam
{

// In-memory point cloud read from a binary PCD v0.7 file. `intensities` is
// empty when the file has no intensity field or it is not float32. `colors`
// ({r, g, b}, parallel to `points`) is empty when the file has no rgb field.
struct PcdCloud
{
  std::vector<std::array<float, 3>> points;
  std::vector<float> intensities;
  std::vector<std::array<std::uint8_t, 3>> colors;
};

// Result of read_pcd(). On success `ok` is true and `cloud` holds the data; on
// failure `ok` is false and `error` carries a short diagnostic.
struct PcdReadResult
{
  bool ok = false;
  PcdCloud cloud;
  std::string error;
};

// Write `points` as a binary PCD v0.7 point cloud: each point carries `x y z`
// as float32, an `intensity` float32 field when `intensities` is non-empty
// AND exactly `points.size()` long (otherwise it is omitted), and an `rgb`
// field when `colors` is non-empty AND exactly `points.size()` long (same
// omission rule). `rgb` uses PCL's packed-float convention — the float32's bit
// pattern is the uint32 `0x00RRGGBB` — which both PCL and three.js' `PCDLoader`
// decode. The body is tightly packed (no struct padding), so the implied field
// offsets match what both PCL (`pcl::io::loadPCDFile`) and three.js'
// `PCDLoader` reconstruct from the header. Mirrors `core::write_tum`'s void
// shape — the caller checks the stream state afterwards. Assumes a
// little-endian host (bagwiz targets x86).
void write_pcd(
  std::ostream & os, std::span<const std::array<float, 3>> points,
  std::span<const float> intensities = {},
  std::span<const std::array<std::uint8_t, 3>> colors = {});

// Read a binary PCD v0.7 point cloud from `is`. Supports the subset produced by
// write_pcd(): FIELDS `x y z` optionally followed by `intensity` and/or `rgb`
// (in that order), TYPE `F`, SIZE 4, COUNT 1, little-endian, DATA binary.
// Returns an error for ASCII data, big-endian data, unsupported field layouts,
// or malformed headers.
PcdReadResult read_pcd(std::istream & is);

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__POINT_CLOUD_IO_HPP_
