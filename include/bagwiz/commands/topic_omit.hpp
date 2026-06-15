// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__TOPIC_OMIT_HPP_
#define BAGWIZ__COMMANDS__TOPIC_OMIT_HPP_

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace bagwiz::commands
{

// Arguments for `bagwiz topic omit`. Populated by TopicCommand's CLI wiring
// (src/commands/topic.cpp) and consumed by run_topic_omit. Kept in a header so
// the run function can be exercised directly from tests without driving the
// CLI parser.
struct TopicOmitArgs
{
  std::filesystem::path input_path;
  // Topic selectors to remove. Each is a literal topic name or a '*' glob;
  // see bagwiz/core/topic_match.hpp for the matching rules.
  std::vector<std::string> topics;
  // Empty: rewrite <input> in place. Set: write the result to this new bag and
  // leave <input> untouched.
  std::optional<std::filesystem::path> output_path;
  // Replace a pre-existing output_path (no effect in in-place mode).
  bool overwrite = false;
};

// Remove every topic matched by any of `args.topics` from `args.input_path`,
// copying all other topics verbatim. Returns a process exit code: 0 on success,
// 1 on any error (input open failure, a selector matching no topic, output
// collision, or a read/write/close error).
int run_topic_omit(const TopicOmitArgs & args);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__TOPIC_OMIT_HPP_
