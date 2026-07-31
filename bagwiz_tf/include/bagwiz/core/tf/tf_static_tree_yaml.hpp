// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__TF__TF_STATIC_TREE_YAML_HPP_
#define BAGWIZ__CORE__TF__TF_STATIC_TREE_YAML_HPP_

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

// Renders a static TF tree as the nested parent -> child -> {x,y,z,roll,pitch,
// yaw} YAML that static-transform publisher configs use, i.e. the inverse of
// reading such a config and broadcasting it. Kept free of I/O so it is pure and
// unit-testable without a bag; `bagwiz tf static dump` decides where the string
// goes.
namespace bagwiz::core
{

// Render `transforms` as YAML: one top-level mapping per parent frame
// (header.frame_id), holding one mapping per child frame (child_frame_id), each
// holding the six scalars `x`, `y`, `z`, `roll`, `pitch`, `yaw`.
//
// Rotations are roll/pitch/yaw in RADIANS via quaternion_to_rpy(), i.e. tf2's
// fixed-axis convention, so feeding them back through
// tf2::Quaternion::setRPY(roll, pitch, yaw) reproduces the input quaternion.
// header.stamp has no place in this schema and is dropped.
//
// Parent groups are ordered breadth-first from the tree's roots (a parent frame
// that is never a child), children in first-seen order within a parent, which
// puts the base frame first and reads top-down. A parent unreachable from any
// root — only possible for a cyclic input, which a valid TF tree never is — is
// appended after the reachable ones so no transform is ever dropped.
//
// Numbers carry 16 significant digits and always show a decimal point (`0.0`,
// never `0`), so a consumer that demands a float does not trip over an integer.
// Frame ids come from the bag and are untrusted: names outside a conservative
// plain-scalar set, or that a YAML reader would resolve as a bool/null rather
// than a string, are emitted as escaped double-quoted scalars.
//
// `source_label` is recorded in a header comment (line breaks are stripped so it
// cannot escape the comment). The result ends with a newline.
std::string emit_static_tf_tree_yaml(
  std::span<const geometry_msgs::msg::TransformStamped> transforms, std::string_view source_label);

// Outcome of parse_static_tf_tree_yaml(). On success `transforms` is set and
// `error` is empty; on any problem `transforms` is empty and `error` explains
// why (unreadable file, wrong shape, missing or unknown key, non-numeric value,
// a frame with two parents, a cycle, ...). Never throws.
struct StaticTfTreeParseResult
{
  std::optional<std::vector<geometry_msgs::msg::TransformStamped>> transforms;
  std::string error;
  // Keys that named a grouping heading rather than a frame: nothing directly
  // under them was a transform, so they parent nothing (see the nesting note on
  // parse_static_tf_tree_yaml). Legal and lossless, but reported so a caller can
  // surface it — an author who meant a chain, not a heading, gets a signal that
  // the key produced no transform. Empty for the two-level form `dump` writes.
  std::vector<std::string> grouping_frames;

  [[nodiscard]] bool ok() const noexcept { return transforms.has_value() && error.empty(); }
};

// Read the YAML emit_static_tf_tree_yaml() writes and rebuild the transforms,
// so that parse -> emit -> parse is a fixed point. Rotations are converted from
// RPY radians back to a quaternion with rpy_to_quaternion() (tf2 fixed-axis), and
// header.stamp is left zero: the schema carries none, and the caller stamps the
// transforms for the bag it is writing them into.
//
// Nesting may go arbitrarily deep, matching the reference publisher: a mapping
// that carries transform fields is an edge from the key enclosing it, and one
// that does not is a further level. So depth beyond two is NOT a chain — it is a
// grouping heading, which lets a large rig config be split into sections:
//
//     sensors:            # a heading; parents nothing, reported in
//       base_link:        # grouping_frames
//         drs_base_link:
//           x: ...        # => base_link -> drs_base_link
//
// Only the level immediately above a transform names its parent.
//
// Otherwise deliberately strict, because this is a hand-edited file at a system
// boundary and a silently-ignored key becomes a silently-wrong sensor pose.
// Rejected: a transform missing any of `x`, `y`, `z`, `roll`, `pitch`, `yaw`, or
// carrying any other key (which is also how a child nested beside those keys is
// caught — the publisher would drop that transform without a word); a value that
// is neither a transform nor child frames; a mapping that is empty; a non-numeric
// value; an empty frame id; a transform at the document root, which has no
// enclosing key to be its parent; a self edge; and any edge set that is not a
// forest (a child with two parents, opposite edges, a cycle — see
// validate_tf_forest). An empty document is an error too: there would be nothing
// to write.
//
// `yaml_path` is opened directly so YAML::Exception can be turned into `error`
// rather than escaping.
[[nodiscard]] StaticTfTreeParseResult parse_static_tf_tree_yaml(
  const std::filesystem::path & yaml_path);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__TF__TF_STATIC_TREE_YAML_HPP_
