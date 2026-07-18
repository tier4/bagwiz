// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__POINTCLOUD__STATIC_EXTRINSIC_HPP_
#define BAGWIZ__CORE__POINTCLOUD__STATIC_EXTRINSIC_HPP_

#include <tf2/buffer_core.hpp>
#include <tf2/time.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <string>
#include <vector>

namespace bagwiz::core::pointcloud
{

// Outcome of resolve_static_extrinsic().
struct StaticExtrinsicResult
{
  // The resolved target_frame <- source_frame transform. Meaningful only when
  // ok(); for identical frames this is the identity (tf2's same-frame fast
  // path, taken after the presence check).
  geometry_msgs::msg::TransformStamped transform;
  // Frames absent from the buffer's TF tree (core::missing_frames), in
  // {target_frame, source_frame} order. Empty when both frames exist.
  std::vector<std::string> missing;
  // lookupTransform's exception detail when both frames exist but the lookup
  // still failed (e.g. no chain between them). Empty on success.
  std::string lookup_error;

  [[nodiscard]] bool ok() const { return missing.empty() && lookup_error.empty(); }
};

// Resolve the static extrinsic `target_frame <- source_frame` from a bag's TF
// tree: absent frames are reported via `missing` (so an identity same-frame
// lookup can never mask an unknown frame), then the transform is read at
// tf2::TimePointZero — static TF is time-invariant, so any populated stamp
// would do. Caller-neutral: the result names only frames, never a command's
// flags; callers own their flag-specific error wording.
[[nodiscard]] StaticExtrinsicResult resolve_static_extrinsic(
  const tf2::BufferCore & buffer, const std::string & target_frame,
  const std::string & source_frame);

}  // namespace bagwiz::core::pointcloud

#endif  // BAGWIZ__CORE__POINTCLOUD__STATIC_EXTRINSIC_HPP_
