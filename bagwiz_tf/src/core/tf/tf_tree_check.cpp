// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tf/tf_tree_check.hpp"

#include "bagwiz/core/tf/tf_forest_check.hpp"

#include <tf2/buffer_core.hpp>
#include <tf2/time.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <optional>
#include <set>
#include <span>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>

namespace bagwiz::core
{

namespace
{

// Below this squared length a quaternion carries no usable orientation (an
// all-zero default-initialised message lands here). Matches the guard in
// tf_transform_format.cpp.
constexpr double kMinQuatLength2 = 1e-12;

// How far a quaternion's squared length may sit from 1 before it stops denoting a
// rotation. tf2 does NOT reject a denormalized quaternion — it stores it and
// tf2::Matrix3x3 then builds its matrix from the raw components without
// normalising (the reason quaternion_to_rpy normalises first), so the transform
// comes out skewed rather than dropped. Silently wrong geometry is worse than a
// missing frame, hence the check.
//
// 1e-6 is chosen to sit between the two: a bag that stored its quaternion as
// float32 is off by ~1e-7 after widening, which must pass, while the skew this
// tolerance still admits is ~1e-6 relative — a tenth of a millimetre over a 100 m
// lever arm.
constexpr double kQuatLength2Tolerance = 1e-6;

// Static entries ignore the query time, so the cache window only has to be big
// enough not to expire them. Matches `tf static calc`.
constexpr auto kBufferCacheTime = std::chrono::hours(24 * 365);

// "TF tree <context>: " — the prefix validate_tf_forest also uses, so every
// message from this module reads the same whichever layer produced it.
std::ostringstream message_prefix(const std::string & context)
{
  std::ostringstream oss;
  oss << "TF tree " << context << ": ";
  return oss;
}

std::string edge_label(const geometry_msgs::msg::TransformStamped & t)
{
  return "'" + t.header.frame_id + "' -> '" + t.child_frame_id + "'";
}

// Layer 1: the per-transform preconditions tf2 enforces by silently dropping the
// transform. Reported here so the message names the frame and the field.
std::optional<std::string> validate_transform_values(
  const geometry_msgs::msg::TransformStamped & t, const std::string & context)
{
  if (t.header.frame_id.empty() || t.child_frame_id.empty()) {
    auto oss = message_prefix(context);
    oss << "a transform has an empty frame id (parent '" << t.header.frame_id << "', child '"
        << t.child_frame_id << "')";
    return oss.str();
  }
  if (t.header.frame_id == t.child_frame_id) {
    auto oss = message_prefix(context);
    oss << "transform " << edge_label(t) << " is its own parent";
    return oss.str();
  }

  const auto & tr = t.transform.translation;
  const auto & q = t.transform.rotation;
  const std::array<std::pair<const char *, double>, 7> fields{
    {{"translation.x", tr.x},
     {"translation.y", tr.y},
     {"translation.z", tr.z},
     {"rotation.x", q.x},
     {"rotation.y", q.y},
     {"rotation.z", q.z},
     {"rotation.w", q.w}}};
  for (const auto & field : fields) {
    if (!std::isfinite(field.second)) {
      auto oss = message_prefix(context);
      oss << "transform " << edge_label(t) << " has a non-finite " << field.first
          << "; tf2 would drop the transform and leave the frame unreachable";
      return oss.str();
    }
  }

  const double length2 = (q.x * q.x) + (q.y * q.y) + (q.z * q.z) + (q.w * q.w);
  if (length2 < kMinQuatLength2) {
    auto oss = message_prefix(context);
    oss << "transform " << edge_label(t)
        << " has a zero-length rotation, which denotes no orientation (a unit quaternion needs "
           "w = 1 for the identity, not 0)";
    return oss.str();
  }
  if (std::abs(length2 - 1.0) > kQuatLength2Tolerance) {
    auto oss = message_prefix(context);
    oss << "transform " << edge_label(t) << " has a rotation of length " << std::sqrt(length2)
        << " rather than 1, so it is not a rotation; tf2 would keep it and skew the transform "
           "rather than reject it";
    return oss.str();
  }
  return std::nullopt;
}

}  // namespace

std::optional<std::string> validate_tf_tree(
  std::span<const geometry_msgs::msg::TransformStamped> transforms, const std::string & context)
{
  if (transforms.empty()) {
    return message_prefix(context).str() + "no transforms to build a tree from";
  }

  std::set<std::pair<std::string, std::string>> edges;
  for (const auto & t : transforms) {
    if (auto err = validate_transform_values(t, context)) {
      return err;
    }
    edges.emplace(t.header.frame_id, t.child_frame_id);
  }

  // Layer 2: the shape of the edge set (unique parent, no opposite edges, no
  // cycle). Same check `bagwiz tf tree` applies to a bag's merged tree.
  if (auto err = validate_tf_forest(edges, context)) {
    return err;
  }

  // Layer 3: tf2 is what will load the result, so let it decide. setTransform
  // returns false for anything it refuses; layer 1 already covers the cases it
  // would only complain about through its own logger.
  tf2::BufferCore buffer{kBufferCacheTime};
  for (const auto & t : transforms) {
    if (!buffer.setTransform(t, "bagwiz", /*is_static=*/true)) {
      auto oss = message_prefix(context);
      oss << "tf2 refused the transform " << edge_label(t);
      return oss.str();
    }
  }

  // And the tree is usable, not merely accepted: every frame must resolve against
  // the root of its own tree. Several roots (a forest) is fine — frames are only
  // ever resolved within their tree, never across two.
  std::unordered_map<std::string, std::string> child_to_parent;
  for (const auto & [parent, child] : edges) {
    child_to_parent.emplace(child, parent);
  }
  for (const auto & edge : edges) {
    const std::string & child = edge.second;
    // validate_tf_forest has ruled out cycles, so walking up terminates.
    std::string root = child;
    for (auto it = child_to_parent.find(root); it != child_to_parent.end();
         it = child_to_parent.find(root)) {
      root = it->second;
    }
    std::string tf2_error;
    if (!buffer.canTransform(root, child, tf2::TimePointZero, &tf2_error)) {
      auto oss = message_prefix(context);
      oss << "frame '" << child << "' does not resolve to its root '" << root << "'";
      if (!tf2_error.empty()) {
        oss << ": " << tf2_error;
      }
      return oss.str();
    }
  }

  return std::nullopt;
}

}  // namespace bagwiz::core
