// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__TF__TF_TREE_CHECK_HPP_
#define BAGWIZ__CORE__TF__TF_TREE_CHECK_HPP_

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <optional>
#include <span>
#include <string>

// Whether a usable tf2 tree can actually be built from a set of transforms.
// Complements the other two checks in this package, which each cover less:
// tf_merge_check.hpp finds contradictions *between sources*, and
// tf_forest_check.hpp validates the shape of a bare edge set. Neither looks at
// the transform VALUES, so neither notices a set that tf2 will refuse at load
// time.
namespace bagwiz::core
{

// Validate that `transforms` would build a tf2 tree, and return the first problem
// found or std::nullopt when they would. `context` names the source for the error
// messages, e.g. "in 'rig.yaml'".
//
// Use this before writing transforms into a bag. Structural validity is not
// enough, because tf2 mishandles two kinds of bad value in ways that surface far
// from their cause:
//
//   * A non-finite component makes tf2::BufferCore DROP the transform (logging
//     TF_NAN_INPUT), so the bag holds a perfectly well-formed /tf_static whose
//     tree is empty the moment anything tries to use it.
//   * A quaternion that is not unit length is KEPT, and tf2::Matrix3x3 then
//     builds its matrix from the raw components without normalising (the reason
//     quaternion_to_rpy normalises first), so the transform comes out skewed.
//     Silently wrong geometry is worse than a missing frame.
//
// Three layers, cheapest and most specific first:
//
//  1. Per transform: non-empty frame ids, no self edge, every translation and
//     rotation component finite, and a rotation of unit length. Checked here
//     rather than left to tf2, both because tf2 does not catch all of it and so
//     the error names the offending frame and field instead of tf2's logger
//     firing mid-write.
//  2. The edge set as a whole, via validate_tf_forest(): unique parent per child,
//     no opposite edges, no cycle.
//  3. tf2 itself, as a backstop for anything the first two do not anticipate:
//     every transform is fed to a tf2::BufferCore as a static entry, and then
//     every frame is resolved against its own tree root. A forest with several
//     roots is fine, as frames are only ever resolved within their own tree.
std::optional<std::string> validate_tf_tree(
  std::span<const geometry_msgs::msg::TransformStamped> transforms, const std::string & context);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__TF__TF_TREE_CHECK_HPP_
