// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/base/str_utils.hpp"

#include <array>
#include <cinttypes>
#include <cstdio>
#include <ctime>
#include <string>
#include <vector>

namespace bagwiz::core
{

std::string format_timestamp(std::int64_t ns)
{
  const auto seconds = static_cast<std::time_t>(ns / 1'000'000'000);
  const auto nanos = static_cast<std::int64_t>(ns % 1'000'000'000);
  std::tm tm_utc{};
  ::gmtime_r(&seconds, &tm_utc);
  std::array<char, 32> buf{};
  std::strftime(buf.data(), buf.size(), "%Y-%m-%d %H:%M:%S", &tm_utc);
  // Keep this formatting fmt-free: bagwiz_base links only rcutils, so the
  // "<stamp>.<nanos> UTC (<seconds>.<nanos>)" shape is rendered with snprintf
  // (zero-padded to 9 digits, matching {:09d}).
  std::array<char, 96> out{};
  std::snprintf(
    out.data(), out.size(), "%s.%09" PRId64 " UTC (%" PRId64 ".%09" PRId64 ")", buf.data(), nanos,
    static_cast<std::int64_t>(seconds), nanos);
  return out.data();
}

std::vector<std::string> split_lines(const std::string & s)
{
  std::vector<std::string> out;
  std::size_t start = 0;
  for (std::size_t i = 0; i < s.size(); ++i) {
    if (s[i] == '\n') {
      out.emplace_back(s.data() + start, i - start);
      start = i + 1;
    }
  }
  if (start < s.size()) {
    out.emplace_back(s.data() + start, s.size() - start);
  }
  return out;
}

std::string join_csv(const std::vector<std::string> & names)
{
  if (names.empty()) {
    return "(none)";
  }
  std::string out;
  for (std::size_t i = 0; i < names.size(); ++i) {
    if (i > 0) {
      out += ", ";
    }
    out += names[i];
  }
  return out;
}

}  // namespace bagwiz::core
