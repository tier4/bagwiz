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

#include <span>
#include <string>
#include <string_view>

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

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__TF__TF_STATIC_TREE_YAML_HPP_
