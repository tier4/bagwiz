// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__TOPIC_RENAME_HPP_
#define BAGWIZ__COMMANDS__TOPIC_RENAME_HPP_

#include <filesystem>
#include <optional>
#include <string>

namespace bagwiz::commands
{

// Arguments for `bagwiz topic rename`. Populated by TopicCommand's CLI wiring
// (src/commands/topic.cpp) and consumed by run_topic_rename. Kept in a header so
// the run function can be exercised directly from tests without driving the CLI
// parser. Unlike `drop` / `keep`, rename is a 1:1 operation: `src_topic` and
// `dst_topic` are literal topic names, not '*' globs.
struct TopicRenameArgs
{
  std::filesystem::path input_path;
  // Existing topic to rename. A literal topic name (no glob); it must match a
  // topic in the bag exactly or the run fails before anything is written.
  std::string src_topic;
  // New name for the topic. Must not already name a topic in the bag, otherwise
  // the rename would collide two distinct declarations onto one name.
  std::string dst_topic;
  // Empty: rewrite <input> in place. Set: write the result to this new bag and
  // leave <input> untouched.
  std::optional<std::filesystem::path> output_path;
  // Replace a pre-existing output_path (no effect in in-place mode).
  bool overwrite = false;
};

// Rename `args.src_topic` to `args.dst_topic` in `args.input_path`, copying
// every other topic and message verbatim (only the name string changes; no
// deserialization or type conversion). Returns a process exit code: 0 on
// success, 1 on any error (input open failure, src not found, dst already
// present, src == dst, output collision, or a read/write/close error).
int run_topic_rename(const TopicRenameArgs & args);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__TOPIC_RENAME_HPP_
