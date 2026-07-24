// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/topic_rename.hpp"

#include "bagwiz/core/bag/bag_copy.hpp"
#include "bagwiz/core/bag/bag_passthrough.hpp"
#include "bagwiz/core/bag/rewrite.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/bag_open.hpp"
#include "bagwiz/io/topics.hpp"

#include <algorithm>
#include <cinttypes>
#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace bagwiz::commands
{

namespace
{

constexpr const char * kLogger = "bagwiz.cmd.topic";

// Declare every input topic verbatim except `src`, which is re-declared under
// `dst` (its type, QoS, and embedded schema preserved). Then stream-copy the
// bag, remapping `src`'s messages onto `dst`. Shared by the in-place and -o
// modes; the writer factory is injected so the rewrite dispatch
// (core::run_bag_rewrite) can supply a tmp path. Returns a process exit code.
int execute_rename_pass(
  const TopicRenameArgs & args, const io::WriterFactory & open_writer,
  const core::RewriteTarget & target)
{
  // Chunk pass-through fast path: chunk bytes reference channels by numeric
  // id only, so a rename rewrites the Channel record while chunks without
  // the renamed channel copy byte-for-byte, preserving the input's chunk
  // compression. Falls back to the decoded stream copy below whenever the
  // input, the target, or the edit is ineligible.
  {
    core::PassthroughEdit edit;
    edit.rename = {{args.src_topic, args.dst_topic}};
    if (const auto pt = core::try_bag_passthrough_rewrite(args.input_path, target, edit, kLogger)) {
      BAGWIZ_LOG_INFO(
        kLogger, "topic rename: '%s' -> '%s'; copied %" PRIu64 " message(s) (%" PRIu64 " renamed).",
        args.src_topic.c_str(), args.dst_topic.c_str(), pt->copied, pt->renamed);
      return 0;
    }
  }

  auto reader = io::open_read_or_log(args.input_path, kLogger);
  if (!reader) {
    return 1;
  }
  // Backfill embedded schemas so MCAP outputs keep self-description for the
  // renamed topic (no-op for single-file readers and SQLite3 inputs). The
  // schema is keyed by message type, not topic name, so it survives the rename
  // unchanged.
  reader->populate_schemas();

  auto writer = io::open_write_or_log(open_writer, kLogger);
  if (!writer) {
    return 1;
  }

  // Declare each topic. `src` is re-declared under `dst` by copying its
  // TopicInfo and replacing only the name (immutable: the reader's entry is
  // left untouched). The writer rejects writes to undeclared topics, so `dst`
  // must be declared before the stream copy below writes under it.
  for (const auto & t : reader->topics()) {
    try {
      if (t.name == args.src_topic) {
        auto renamed = t;
        renamed.name = args.dst_topic;
        writer->declare_topic(renamed);
      } else {
        writer->declare_topic(t);
      }
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "declare_topic failed for '%s': %s", t.name.c_str(), e.what());
      return 1;
    }
  }

  core::BagCopyRenameCounts counts;
  try {
    const std::unordered_map<std::string, std::string> rename{{args.src_topic, args.dst_topic}};
    counts = core::bag_copy_renamed(
      *reader, *writer, rename, "topic rename", core::pipeline::BackendKind::Pipelined);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Stream copy from %s failed: %s", args.input_path.c_str(), e.what());
    return 1;
  }

  if (!io::close_writer_or_log(*writer, kLogger)) {
    return 1;
  }

  BAGWIZ_LOG_INFO(
    kLogger, "topic rename: '%s' -> '%s'; copied %" PRIu64 " message(s) (%" PRIu64 " renamed).",
    args.src_topic.c_str(), args.dst_topic.c_str(), counts.copied, counts.renamed);
  return 0;
}

}  // namespace

int run_topic_rename(const TopicRenameArgs & args)
{
  // 0. Guard the public entry point. The CLI marks both names ->required(), but
  //    run_topic_rename is also called directly from tests; reject empty names
  //    and a no-op self-rename before any writer (or in-place tmp) is created.
  if (args.src_topic.empty() || args.dst_topic.empty()) {
    BAGWIZ_LOG_ERROR(
      kLogger, "topic rename: source and destination topics must both be non-empty.");
    return 1;
  }
  if (args.src_topic == args.dst_topic) {
    BAGWIZ_LOG_ERROR(
      kLogger, "topic rename: source and destination are identical ('%s'); nothing to rename.",
      args.src_topic.c_str());
    return 1;
  }

  // 1. Resolve the names against the bag's topic list up front, so a missing
  //    source or a colliding destination fails the run before any writer (or
  //    in-place tmp) is created. The topic list is snapshotted into a vector
  //    because the reader's span is invalidated once the reader is destroyed
  //    below.
  std::vector<std::string> topic_names;
  {
    auto reader = io::open_read_or_log(args.input_path, kLogger);
    if (!reader) {
      return 1;
    }
    topic_names = io::snapshot_topic_names(*reader);
  }

  const bool src_present =
    std::find(topic_names.begin(), topic_names.end(), args.src_topic) != topic_names.end();
  if (!src_present) {
    BAGWIZ_LOG_ERROR(
      kLogger, "topic '%s' not found in %s", args.src_topic.c_str(), args.input_path.c_str());
    return 1;
  }

  const bool dst_present =
    std::find(topic_names.begin(), topic_names.end(), args.dst_topic) != topic_names.end();
  if (dst_present) {
    BAGWIZ_LOG_ERROR(
      kLogger, "destination topic '%s' already exists in %s; rename would collide.",
      args.dst_topic.c_str(), args.input_path.c_str());
    return 1;
  }

  // 2. -o vs in-place dispatch, shared with the other rewrite-style commands:
  //    -o writes a new bag and leaves <input> untouched; otherwise <input> is
  //    rewritten atomically via a sibling tmp, preserving its storage format
  //    and layout.
  core::BagRewriteOptions rewrite_opts;
  rewrite_opts.logger = kLogger;
  rewrite_opts.format_unknown_error =
    "topic rename: could not detect storage format of input bag '%s'.";
  rewrite_opts.pass_failed_error = "topic rename: pass failed; aborting in-place swap";
  rewrite_opts.inherit_output_format = true;
  return core::run_bag_rewrite(
    args.input_path, args.output_path, args.overwrite, rewrite_opts,
    [&](const io::WriterFactory & open_writer, const core::RewriteTarget & target) {
      return execute_rename_pass(args, open_writer, target);
    });
}

}  // namespace bagwiz::commands
