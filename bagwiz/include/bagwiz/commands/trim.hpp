// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__TRIM_HPP_
#define BAGWIZ__COMMANDS__TRIM_HPP_

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace bagwiz::commands
{

// Arguments for `bagwiz trim`. Populated by TrimCommand's CLI wiring
// (src/commands/trim.cpp) and consumed by run_trim. Kept in a header so the
// run function can be exercised directly from tests without driving the CLI
// parser.
struct TrimArgs
{
  std::filesystem::path input_path;
  // Window bounds as offsets from the bag's start time, kept as the raw CLI
  // strings. `start`, `end`, and `both` are parsed with parse_bound_or_log: a
  // value carrying the `msg` unit is a message count (parsed to an integer via
  // std::from_chars), otherwise it is a duration parsed with
  // core::parse_duration_ns under DurationUnitPolicy::RequireUnit (a bare number
  // is rejected). `duration` is time-only, parsed with core::parse_duration_ns.
  // `end` and `duration` are mutually exclusive, and `both` — shorthand for
  // trimming the same offset from each end of the bag — excludes the other
  // three. At least one window input must be set (start, end, duration, both,
  // or align).
  std::optional<std::string> start;
  std::optional<std::string> end;
  std::optional<std::string> duration;
  std::optional<std::string> both;
  // Topic selectors (literal names or '*' globs, as in `topic drop -t`): trim
  // to the selected topics' common time span — from their latest first
  // message to their earliest last message, both boundary messages included.
  // Mutually exclusive with the offset flags above.
  std::vector<std::string> align;
  // Reference clock for the window: "header" (default) evaluates bounds and
  // the per-message keep decision on header.stamp, falling back to receive
  // time for messages without a usable stamp (headerless type, stamp == 0, or
  // unresolvable definition); "recv" uses the record time and keeps the
  // storage-index pushdown fast path.
  std::string stamp = "header";
  // Empty: rewrite <input> in place. Set: write the result to this new bag and
  // leave <input> untouched.
  std::optional<std::filesystem::path> output_path;
  // Replace a pre-existing output_path (no effect in in-place mode).
  bool overwrite = false;
};

// Copy only the messages inside the half-open window [start, end) — resolved
// against the bag's time extent — to the output bag, declaring every topic
// verbatim. Returns a process exit code: 0 on success, 1 on any error (bad
// window arguments, input open failure, no time extent, start past the bag
// end, output collision, or a read/write/close error).
int run_trim(const TrimArgs & args);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__TRIM_HPP_
