// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__TF_STATIC_INJECT_HPP_
#define COMMANDS__TF_STATIC_INJECT_HPP_

#include "bagwiz/core/tf/tf_static_collect.hpp"
#include "bagwiz/io/bag_open.hpp"

#include <filesystem>
#include <string>
#include <vector>

// CLI-internal (not installed): the one rewrite pass that writes latched static
// TF topics into a bag, shared by `bagwiz tf static cp` (transforms read from a
// donor bag) and `bagwiz tf static join` (transforms read from a YAML config).
// Both differ only in where the transforms come from, and getting the write
// ORDER wrong is a silent, hard-to-spot bug, so the pass itself lives in one
// place. Follows the src-local shared-header idiom of traj_common.hpp /
// pcd_concat_common.hpp.
namespace bagwiz::commands
{

// Per-caller knobs for inject_static_tf_pass().
struct StaticTfInjectOptions
{
  // Logger tag for every diagnostic the pass emits, e.g.
  // "bagwiz.cmd.tf.static.cp".
  const char * logger = "bagwiz.cmd.tf.static";
  // Command name used in the summary line, e.g. "tf static cp".
  std::string label;
  // Passed to core::bag_copy_filtered as its profile_label, which names the
  // per-stage bottleneck report under BAGWIZ_PROFILE. Kept separate from
  // `label` because that report's labels are identifiers, e.g. "tf_static_cp".
  std::string profile_label;
  // Whether a destination topic that already carries messages may have them
  // dropped and replaced. This is the user's -w/--overwrite for `cp` and
  // --force for `join`; without it such a collision aborts the pass.
  bool replace_existing_topic = false;
};

// One full pass over `dst_path`: plan each topic in `topics` against what the
// destination already has, declare topics on the writer, write one latched
// TFMessage per entry stamped at the destination's earliest message time, then
// stream-copy everything else through (suppressing the topics being replaced).
//
// The injected messages are written BEFORE the stream copy. They carry the
// bag's lowest timestamp, so appending them would put them at the highest
// storage position — the only rows whose physical order disagrees with their
// time. A consumer that reads a .db3 in row order rather than by timestamp
// (Foxglove's readers issue their message query without an ORDER BY) would then
// receive the static TF last, after everything it is supposed to precede.
//
// `open_writer` is the factory handed in by core::run_bag_rewrite, so the same
// pass serves both in-place (tmp path) and explicit -o modes. Returns the
// process exit code: 0 on success, 1 after logging any failure (bag could not be
// opened, an unresolved topic/type conflict, a serialize/declare/write failure,
// or an I/O error).
int inject_static_tf_pass(
  const std::filesystem::path & dst_path, const std::vector<core::StaticTopicTransforms> & topics,
  const StaticTfInjectOptions & options, const io::WriterFactory & open_writer);

}  // namespace bagwiz::commands

#endif  // COMMANDS__TF_STATIC_INJECT_HPP_
