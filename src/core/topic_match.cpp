// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/topic_match.hpp"

#include <cstddef>
#include <span>
#include <string>
#include <string_view>

namespace bagwiz::core
{

bool topic_glob_match(std::string_view pattern, std::string_view topic)
{
  // Iterative wildcard match with backtracking. '*' is the only metacharacter;
  // it matches any run of characters (including '/' and the empty string).
  // `star` remembers the position of the most recent '*' in the pattern, and
  // `resume` the topic position to retry from when a later literal mismatch
  // forces that '*' to absorb one more character.
  std::size_t p = 0;
  std::size_t t = 0;
  std::size_t star = std::string_view::npos;
  std::size_t resume = 0;

  while (t < topic.size()) {
    if (p < pattern.size() && pattern[p] == '*') {
      star = p;
      resume = t;
      ++p;
    } else if (p < pattern.size() && pattern[p] == topic[t]) {
      ++p;
      ++t;
    } else if (star != std::string_view::npos) {
      // Let the last '*' swallow one more character of the topic and retry.
      p = star + 1;
      ++resume;
      t = resume;
    } else {
      return false;
    }
  }

  // Any pattern tail left over must be all '*' for the match to hold.
  while (p < pattern.size() && pattern[p] == '*') {
    ++p;
  }
  return p == pattern.size();
}

TopicPatternResolution resolve_topic_patterns(
  std::span<const std::string> patterns, std::span<const std::string> topic_names)
{
  TopicPatternResolution result;
  for (const auto & pattern : patterns) {
    bool any = false;
    for (const auto & name : topic_names) {
      if (topic_glob_match(pattern, name)) {
        result.matched.insert(name);
        any = true;
      }
    }
    if (!any) {
      result.unmatched.push_back(pattern);
    }
  }
  return result;
}

}  // namespace bagwiz::core
