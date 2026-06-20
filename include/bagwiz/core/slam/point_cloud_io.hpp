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
#include <ostream>
#include <span>

// GLIM-free point-cloud writers used by `bagwiz slam run`. Kept free of GLIM /
// Eigen / gtsam_points types so the writer (and its test) build in every
// configuration, not only when BAGWIZ_WITH_SLAM is on.
namespace bagwiz::core::slam
{

// Write `points` as a binary PCD v0.7 point cloud: each point carries `x y z`
// as float32, and an `intensity` float32 field when `intensities` is non-empty
// AND exactly `points.size()` long (otherwise it is omitted). The body is
// tightly packed (no struct padding), so the implied field offsets match what
// both PCL (`pcl::io::loadPCDFile`) and three.js' `PCDLoader` reconstruct from
// the header. Mirrors `core::write_tum`'s void shape — the caller checks the
// stream state afterwards. Assumes a little-endian host (bagwiz targets x86).
void write_pcd(
  std::ostream & os, std::span<const std::array<float, 3>> points,
  std::span<const float> intensities = {});

}  // namespace bagwiz::core::slam

#endif  // BAGWIZ__CORE__SLAM__POINT_CLOUD_IO_HPP_
