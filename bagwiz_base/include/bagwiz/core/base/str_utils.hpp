// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__BASE__STR_UTILS_HPP_
#define BAGWIZ__CORE__BASE__STR_UTILS_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace bagwiz::core
{

// UTC timestamp with nanosecond precision from a nanosecond epoch count,
// e.g. "2023-11-14 22:13:20.123456789 UTC (1700000000.123456789)". Used by the
// interactive walkers' header rows.
std::string format_timestamp(std::int64_t ns);

// Split a '\n'-delimited string into owned lines (a trailing '\n' does not
// produce an empty final element; interior empty lines are kept).
std::vector<std::string> split_lines(const std::string & s);

// Comma-joined list for human-readable messages; "(none)" when empty.
std::string join_csv(const std::vector<std::string> & names);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__BASE__STR_UTILS_HPP_
