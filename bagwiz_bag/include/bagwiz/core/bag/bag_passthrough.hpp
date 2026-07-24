// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__BAG__BAG_PASSTHROUGH_HPP_
#define BAGWIZ__CORE__BAG__BAG_PASSTHROUGH_HPP_

#include "bagwiz/core/bag/rewrite.hpp"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>

// Bag-level orchestration of the mcap chunk pass-through rewrite
// (io::mcap_passthrough_rewrite): decide whether a command's edit and the
// resolved write target are eligible for the record-level fast path, run the
// engine shard-to-shard, and emit the rosbag2 metadata.yaml for directory
// outputs. The pure-copy rewrite commands (topic drop/keep/rename, trim)
// call try_bag_passthrough_rewrite() first and fall back to the decoded
// read/process/write pipeline when it declines.
namespace bagwiz::core
{

// The edit, in command-level terms. Mirrors io::McapPassthroughEdit;
// suppress_topics matches bag_copy_filtered's "suppress" wording.
struct PassthroughEdit
{
  std::unordered_set<std::string> suppress_topics;
  std::unordered_map<std::string, std::string> rename;  // old topic -> new topic
  // Half-open [start_ns, end_ns) on message log time (ReadFilter's
  // convention). Unset bound = unbounded on that side.
  std::optional<std::int64_t> start_ns;
  std::optional<std::int64_t> end_ns;
};

// The counts a command needs for its summary log lines.
struct PassthroughCounts
{
  std::uint64_t copied = 0;   // messages written to the output
  std::uint64_t renamed = 0;  // subset of `copied` written under a new name
};

// Try the chunk pass-through for one rewrite. Returns the counts when the
// fast path ran; nullopt when the input, the target, or the edit needs the
// decoded pipeline (the caller then runs its existing pass against the same
// target, whose writer factory recreates anything this function cleaned up).
//
// Declines when: BAGWIZ_PASSTHROUGH is set to off/0/false/no; the target
// requests output splitting; the input is not mcap storage or is a
// multi-shard directory bag; the input uses rosbag2 MESSAGE-mode
// (per-message) compression — the decoded pipeline decompresses those
// payloads, so a verbatim copy would diverge; the resolved output format is
// not mcap; or the engine itself declines (see mcap_passthrough_rewrite).
// Engine-level declines are logged at DEBUG with the reason; a run that
// engages logs one INFO line with the chunk counts.
//
// I/O errors on an eligible input propagate as exceptions, which in in-place
// mode aborts the swap and leaves the input untouched.
std::optional<PassthroughCounts> try_bag_passthrough_rewrite(
  const std::filesystem::path & input_path, const RewriteTarget & target,
  const PassthroughEdit & edit, const char * logger);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__BAG__BAG_PASSTHROUGH_HPP_
