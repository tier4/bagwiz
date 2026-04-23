// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/topic_pattern.hpp"

#include <string>
#include <string_view>

namespace bagwiz::core
{

namespace
{

bool contains_wildcard(std::string_view pattern)
{
  return pattern.find('*') != std::string_view::npos;
}

// Translate the user-supplied glob into an ECMAScript regex fragment. Only
// '*' is a metacharacter (any char except '/'); every other regex-special
// character is escaped so literal topic names with dots, plus signs, etc.
// behave as written.
std::string glob_to_regex(std::string_view pattern)
{
  std::string out;
  out.reserve(pattern.size() * 2);
  for (const char c : pattern) {
    switch (c) {
      case '*':
        out += "[^/]*";
        break;
      case '.':
      case '\\':
      case '+':
      case '?':
      case '(':
      case ')':
      case '[':
      case ']':
      case '{':
      case '}':
      case '|':
      case '^':
      case '$':
        out += '\\';
        out += c;
        break;
      default:
        out += c;
    }
  }
  return out;
}

std::string build_regex(std::string_view pattern)
{
  const bool anchor_start = !pattern.empty() && pattern.front() == '/';
  const bool anchor_end = contains_wildcard(pattern);
  std::string re;
  if (anchor_start) {
    re += '^';
  }
  re += glob_to_regex(pattern);
  if (anchor_end) {
    re += '$';
  }
  return re;
}

}  // namespace

TopicPattern::TopicPattern(std::string_view pattern)
: match_all_(pattern.empty()),
  regex_source_(match_all_ ? std::string{} : build_regex(pattern)),
  regex_(regex_source_)
{
}

bool TopicPattern::matches(const std::string & topic) const
{
  if (match_all_) {
    return true;
  }
  return std::regex_search(topic, regex_);
}

}  // namespace bagwiz::core
