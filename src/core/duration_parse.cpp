// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/duration_parse.hpp"

#include <cctype>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <string>

namespace bagwiz::core
{

namespace
{

std::string_view trim(std::string_view s)
{
  while (!s.empty() && (std::isspace(static_cast<unsigned char>(s.front())) != 0)) {
    s.remove_prefix(1);
  }
  while (!s.empty() && (std::isspace(static_cast<unsigned char>(s.back())) != 0)) {
    s.remove_suffix(1);
  }
  return s;
}

// Nanoseconds per unit, or -1 for an unknown unit. Empty defaults to ms.
std::int64_t unit_factor_ns(std::string_view unit)
{
  if (unit.empty() || unit == "ms") {
    return 1'000'000;
  }
  if (unit == "ns") {
    return 1;
  }
  if (unit == "us" || unit == "µs") {  // "us" or "µs" (UTF-8 0xC2 0xB5)
    return 1'000;
  }
  if (unit == "s") {
    return 1'000'000'000;
  }
  return -1;
}

}  // namespace

// cppcheck-suppress passedByValue  // string_view is the canonical by-value idiom
std::optional<std::int64_t> parse_duration_ns(std::string_view text)
{
  const std::string_view trimmed = trim(text);
  if (trimmed.empty()) {
    return std::nullopt;
  }

  // strtod needs a NUL-terminated buffer; copy the trimmed span.
  const std::string buf(trimmed);
  const char * begin = buf.c_str();
  char * num_end = nullptr;
  const double value = std::strtod(begin, &num_end);
  if (num_end == begin || !std::isfinite(value)) {
    return std::nullopt;  // no leading number, or NaN/Inf (strtod parses "nan"/"inf")
  }

  const std::string_view unit = trim(std::string_view(num_end, buf.c_str() + buf.size() - num_end));
  const std::int64_t factor = unit_factor_ns(unit);
  if (factor < 0) {
    return std::nullopt;  // unknown unit / trailing garbage
  }

  const double ns = value * static_cast<double>(factor);
  if (
    !std::isfinite(ns) || ns < static_cast<double>(std::numeric_limits<std::int64_t>::min()) ||
    ns > static_cast<double>(std::numeric_limits<std::int64_t>::max())) {
    return std::nullopt;  // out of the representable int64 nanosecond range
  }
  return static_cast<std::int64_t>(std::llround(ns));
}

}  // namespace bagwiz::core
