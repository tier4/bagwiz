// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__PCD_CONCAT_HPP_
#define BAGWIZ__COMMANDS__PCD_CONCAT_HPP_

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace bagwiz::commands
{

// Parsed arguments for `bagwiz pcd concat`. See docs/commands/pcd.md and
// .claude/plans/pcd-concat.spec.md for the full behaviour.
struct PcdConcatArgs
{
  std::filesystem::path input_path;                  // <input> bag
  std::string output_topic;                          // <output_topic_name> (the new topic)
  std::vector<std::string> pcd_topics;               // --pcd (>= 2)
  std::optional<std::string> frame;                  // --frame; empty => default base_link
  std::optional<std::filesystem::path> output_path;  // -o/--output; empty => in-place
  std::optional<std::string>
    tolerance;  // --tolerance (num + unit ns/us/ms/s, def ms); auto if empty
  std::vector<std::string> stamp_offsets;  // --stamp-offset "topic=value" entries
  bool drop_inputs = false;                // --drop-inputs (default: keep)
  bool force = false;                      // --force (output topic name collision)
  bool overwrite = false;                  // -w/--overwrite (existing -o path)
  std::optional<int> threads;              // -j/--threads; empty => default 8
};

// Execute `bagwiz pcd concat`. Returns a process exit code.
int run_pcd_concat(const PcdConcatArgs & args);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__PCD_CONCAT_HPP_
