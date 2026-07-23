// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__BASE__DURATION_PARSE_HPP_
#define BAGWIZ__CORE__BASE__DURATION_PARSE_HPP_

#include <cstdint>
#include <optional>
#include <string_view>

namespace bagwiz::core
{

// How parse_duration_ns treats a number without a unit suffix.
enum class DurationUnitPolicy {
  DefaultMs,    // bare number is interpreted as milliseconds (historical default)
  RequireUnit,  // bare number is a parse failure
};

// Parse a signed duration with a unit suffix into nanoseconds.
//
// Grammar:  [+-]? number [unit]
//   number : a decimal, optionally fractional (e.g. 50, -50, 0.05, 1.5)
//   unit   : one of "ns", "us", "µs", "ms", "s". A missing unit follows
//            `unit_policy`: DefaultMs reads the number as "ms", RequireUnit
//            rejects it (for options where a bare number would be ambiguous,
//            e.g. `trim --start`).
//
// The value is added to a header.stamp for matching, so the sign is literal:
// "-50ms" is -50e6 ns, "50ms" is +50e6 ns, "500ns" is 500 ns. Surrounding
// whitespace is ignored. Returns std::nullopt on any parse failure (missing
// number, unknown unit, missing unit under RequireUnit, or trailing garbage).
[[nodiscard]] std::optional<std::int64_t> parse_duration_ns(
  std::string_view text, DurationUnitPolicy unit_policy = DurationUnitPolicy::DefaultMs);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__BASE__DURATION_PARSE_HPP_
