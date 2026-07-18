// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__BASE__TOPIC_MATCH_HPP_
#define BAGWIZ__CORE__BASE__TOPIC_MATCH_HPP_

#include <span>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

// Glob matching for topic-name selectors. The only wildcard is '*', which
// matches any run of characters — including '/' and the empty string; every
// other character matches literally. A pattern without '*' is therefore an
// exact topic-name match. Used by `bagwiz topic drop` to expand the
// user-supplied selectors (e.g. "/sensing/*", "*/image_raw", "*") against a
// bag's topic list.
//
// Pure functions — no I/O. The caller owns reading the bag's topic list.
namespace bagwiz::core
{

// True when `pattern` matches `topic` under the '*'-only glob rules above.
[[nodiscard]] bool topic_glob_match(std::string_view pattern, std::string_view topic);

// Outcome of expanding a set of selector patterns against a topic list.
struct TopicPatternResolution
{
  // Topic names matched by at least one pattern (deduplicated across patterns).
  std::unordered_set<std::string> matched;
  // Patterns that matched no topic, preserved in input order (with duplicates).
  // Surfaced so the caller can fail fast on a typo'd selector instead of
  // silently rewriting a bag with nothing removed.
  std::vector<std::string> unmatched;
};

// Expand `patterns` against `topic_names`. A topic enters `matched` when any
// pattern matches it; a pattern enters `unmatched` when it matches no topic.
[[nodiscard]] TopicPatternResolution resolve_topic_patterns(
  std::span<const std::string> patterns, std::span<const std::string> topic_names);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__BASE__TOPIC_MATCH_HPP_
