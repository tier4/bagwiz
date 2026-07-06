// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__POINTCLOUD__CLOUD_TRANSFORM_HPP_
#define BAGWIZ__CORE__POINTCLOUD__CLOUD_TRANSFORM_HPP_

#include "bagwiz/core/pointcloud/pointcloud2.hpp"

#include <array>
#include <string>

namespace bagwiz::core::pointcloud
{

// A rigid SE(3) transform applied to point coordinates as p' = R * p + t.
// `rotation` is row-major 3x3 (R[0..2] = first row). GLIM/Eigen-free so this
// layer stays testable in every build; the command layer converts the TF2 /
// quaternion extrinsic into this plain form.
struct RigidTransform
{
  std::array<double, 9> rotation{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  std::array<double, 3> translation{0.0, 0.0, 0.0};

  // True when this is (numerically) the identity — transform_cloud_xyz can then
  // skip the whole point sweep.
  [[nodiscard]] bool is_identity() const;
};

struct CloudTransformResult
{
  bool ok = false;
  std::string error;
};

// Transform the xyz of every point in `cloud.data` IN PLACE by `tf`.
//
// Requires x/y/z fields present, of the SAME floating type (all FLOAT32 or all
// FLOAT64), each with count == 1, and `!cloud.is_bigendian` (big-endian point
// data is rejected). Non-finite points (any of x/y/z NaN or Inf) are passed
// through unchanged — they carry no position to rotate. All other field bytes
// (intensity/ring/time/…) are untouched.
//
// On failure returns ok == false with a message and leaves `cloud` unmodified
// up to the point of failure (failures are detected before the sweep begins).
CloudTransformResult transform_cloud_xyz(PointCloud2 & cloud, const RigidTransform & tf);

}  // namespace bagwiz::core::pointcloud

#endif  // BAGWIZ__CORE__POINTCLOUD__CLOUD_TRANSFORM_HPP_
