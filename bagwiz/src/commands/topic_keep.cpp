// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/topic_keep.hpp"

#include "bagwiz/core/bag/bag_copy.hpp"
#include "bagwiz/core/bag/rewrite.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/base/topic_match.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/bag_open.hpp"
#include "bagwiz/io/topics.hpp"

#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

namespace bagwiz::commands
{

namespace
{

constexpr const char * kLogger = "bagwiz.cmd.topic";

// Declare only the topics in `keep`, then stream-copy the bag with every other
// topic suppressed (dropped messages). Shared by the in-place and -o modes; the
// writer factory is injected so the rewrite dispatch (core::run_bag_rewrite)
// can supply a tmp path. Returns a process exit code.
int execute_keep_pass(
  const TopicKeepArgs & args, const std::unordered_set<std::string> & keep,
  const io::WriterFactory & open_writer)
{
  auto reader = io::open_read_or_log(args.input_path, kLogger);
  if (!reader) {
    return 1;
  }
  // Backfill embedded schemas so MCAP outputs keep self-description for the
  // surviving topics (no-op for single-file readers and SQLite3 inputs).
  reader->populate_schemas();

  auto writer = io::open_write_or_log(open_writer, kLogger);
  if (!writer) {
    return 1;
  }

  // A topic outside `keep` disappears entirely from the output: it is neither
  // declared nor carries any message. Every kept topic is declared verbatim,
  // and the rest are gathered into the suppress set for the stream copy.
  std::unordered_set<std::string> suppress;
  std::size_t kept = 0;
  for (const auto & t : reader->topics()) {
    if (keep.count(t.name) == 0) {
      suppress.insert(t.name);
      continue;
    }
    try {
      writer->declare_topic(t);
      ++kept;
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "declare_topic failed for '%s': %s", t.name.c_str(), e.what());
      return 1;
    }
  }

  // Push the keep set down to the reader before the copy starts: the indexed
  // MCAP path prunes whole chunks that carry no kept topic (and the SQLite
  // backend filters in SQL), so a sparse keep no longer reads and decompresses
  // the entire bag. The push-down stays at the command level on purpose:
  // bag_copy_filtered must not install reader filters itself, because callers
  // like `trim` set a time-range filter on the reader first and set_filter
  // replaces the whole ReadFilter.
  io::ReadFilter filter;
  filter.topics.assign(keep.begin(), keep.end());
  reader->set_filter(filter);

  // The reader never surfaces the suppressed messages now, so the copy loop
  // cannot count them; report the suppressed total from the bag's statistics
  // instead (0, after the backend's own warning, when the input carries none).
  const auto suppressed = core::count_topic_messages(*reader, suppress);
  if (!suppressed.has_value()) {
    BAGWIZ_LOG_WARN(
      kLogger, "Could not compute the suppressed message count for %s; reporting 0.",
      args.input_path.c_str());
  }

  core::BagCopyCounts counts;
  try {
    counts = core::bag_copy_filtered(
      *reader, *writer, suppress, "topic keep", core::pipeline::BackendKind::Pipelined);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Stream copy from %s failed: %s", args.input_path.c_str(), e.what());
    return 1;
  }

  if (!io::close_writer_or_log(*writer, kLogger)) {
    return 1;
  }

  BAGWIZ_LOG_INFO(
    kLogger,
    "topic keep: kept %zu topic(s), dropped %zu; copied %" PRIu64 " message(s), suppressed %" PRId64
    ".",
    kept, suppress.size(), counts.copied, suppressed.value_or(0));
  return 0;
}

}  // namespace

int run_topic_keep(const TopicKeepArgs & args)
{
  // 0. Guard the public entry point. The CLI marks <topics> ->required(), but
  //    run_topic_keep is also called directly from tests; an empty selector
  //    list would otherwise slip past the unmatched-pattern check below and
  //    silently keep nothing (an empty bag). Fail fast instead.
  if (args.topics.empty()) {
    BAGWIZ_LOG_ERROR(kLogger, "topic keep: no topic selector given; nothing to keep.");
    return 1;
  }

  // 1. Resolve the selectors against the bag's topic list up front, so a typo'd
  //    or non-matching selector fails the run before any writer (or in-place
  //    tmp) is created. The topic list is snapshotted into a vector because the
  //    reader's span is invalidated once the reader is destroyed below.
  std::vector<std::string> topic_names;
  {
    auto reader = io::open_read_or_log(args.input_path, kLogger);
    if (!reader) {
      return 1;
    }
    topic_names = io::snapshot_topic_names(*reader);
  }

  const auto resolution = core::resolve_topic_patterns(args.topics, topic_names);
  if (!resolution.unmatched.empty()) {
    for (const auto & pattern : resolution.unmatched) {
      BAGWIZ_LOG_ERROR(
        kLogger, "selector '%s' matched no topic in %s", pattern.c_str(), args.input_path.c_str());
    }
    return 1;
  }

  // The empty-selector guard and the unmatched-pattern return above together
  // ensure every selector matched at least one topic, so `keep` is non-empty.
  const auto & keep = resolution.matched;
  if (keep.size() == topic_names.size()) {
    BAGWIZ_LOG_WARN(
      kLogger, "all %zu topic(s) matched; the output bag will keep every topic.",
      topic_names.size());
  }

  // 2. -o vs in-place dispatch, shared with the other rewrite-style commands:
  //    -o writes a new bag and leaves <input> untouched; otherwise <input> is
  //    rewritten atomically via a sibling tmp, preserving its storage format
  //    and layout.
  core::BagRewriteOptions rewrite_opts;
  rewrite_opts.logger = kLogger;
  rewrite_opts.format_unknown_error =
    "topic keep: could not detect storage format of input bag '%s'.";
  rewrite_opts.pass_failed_error = "topic keep: pass failed; aborting in-place swap";
  rewrite_opts.inherit_output_format = true;
  return core::run_bag_rewrite(
    args.input_path, args.output_path, args.overwrite, rewrite_opts,
    [&](const io::WriterFactory & open_writer) {
      return execute_keep_pass(args, keep, open_writer);
    });
}

}  // namespace bagwiz::commands
