// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__TF__TF_FOREST_CHECK_HPP_
#define BAGWIZ__CORE__TF__TF_FOREST_CHECK_HPP_

#include <optional>
#include <set>
#include <string>
#include <utility>

// Structural validation of a bare parent->child edge set, independent of where
// the edges came from (bag topics, a YAML config, ...). Complements
// tf_merge_check.hpp, which detects contradictions *between sources*; this
// checks the shape of the union.
namespace bagwiz::core
{

// Validates an edge set as a forest: unique parent per child, no A→B together
// with B→A, no self edges, no cycles. Returns the first problem found, or
// std::nullopt when the set is a valid forest. `context` describes the source
// for the error messages, e.g. "for topic '/tf'", "for the merged topics", or
// "in 'rig.yaml'".
//
// Each element is {parent, child}. A std::set is taken rather than a vector
// because duplicates carry no meaning here and the opposite-edge check needs
// lookup; callers holding a vector should collect it into a set first.
std::optional<std::string> validate_tf_forest(
  const std::set<std::pair<std::string, std::string>> & edges, const std::string & context);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__TF__TF_FOREST_CHECK_HPP_
