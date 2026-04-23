// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__TOPIC_PATTERN_HPP_
#define BAGWIZ__CORE__TOPIC_PATTERN_HPP_

#include <regex>
#include <string>
#include <string_view>

namespace bagwiz::core
{

// Filter for topic names supplied by users on the command line.
//
// Pattern grammar (chosen to match the three common cases without surprise):
//   * empty               -> match every topic
//   * starts with '/'     -> anchored at the start of the topic name
//   * does not start '/'  -> unanchored; matches anywhere in the topic name
//   * '*' as a wildcard   -> matches any characters except '/' (single
//                            segment). When a pattern contains any wildcard
//                            it is additionally anchored at the topic end.
//
// Examples against a topic set like {/sensing/lidar/front/points,
// /sensing/lidar/nebula_packets, /perception/object}:
//   /sensing             -> prefix match, keeps /sensing/*
//   lidar/front          -> substring match
//   /\*/nebula_packets   -> exact match of /<one-segment>/nebula_packets
class TopicPattern
{
public:
  // Construct a matcher from a user-supplied pattern. An empty pattern is
  // treated as "match everything". Throws std::regex_error if the pattern
  // translates to an invalid regex (shouldn't happen for user input because
  // the translator escapes regex metacharacters, but callers may want to
  // surface the error).
  explicit TopicPattern(std::string_view pattern);

  // True iff this matcher was built from an empty pattern, in which case
  // matches() returns true for every input.
  bool match_all() const { return match_all_; }

  // True iff `topic` is kept by this filter.
  bool matches(const std::string & topic) const;

  // Expose the compiled regex source for logging / debugging.
  const std::string & regex_source() const { return regex_source_; }

private:
  bool match_all_;
  std::string regex_source_;
  std::regex regex_;
};

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__TOPIC_PATTERN_HPP_
