// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/topic_omit.hpp"

#include "bagwiz/core/bag_copy.hpp"
#include "bagwiz/core/bag_inplace.hpp"
#include "bagwiz/core/logging.hpp"
#include "bagwiz/core/output_path.hpp"
#include "bagwiz/core/topic_match.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace bagwiz::commands
{

namespace
{

constexpr const char * kLogger = "bagwiz.cmd.topic";

// Declare every input topic except those in `omit`, then stream-copy the bag
// with the same set suppressed (dropped messages). Shared by the in-place and
// -o modes; the writer factory is injected so write_bag_inplace can supply a
// tmp path. Returns a process exit code.
int execute_omit_pass(
  const TopicOmitArgs & args, const std::unordered_set<std::string> & omit,
  const std::function<std::unique_ptr<io::BagWriter>()> & open_writer)
{
  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(args.input_path);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Failed to open %s: %s", args.input_path.c_str(), e.what());
    return 1;
  }
  // Backfill embedded schemas so MCAP outputs keep self-description for the
  // surviving topics (no-op for single-file readers and SQLite3 inputs).
  reader->populate_schemas();

  std::unique_ptr<io::BagWriter> writer;
  try {
    writer = open_writer();
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Failed to open output writer: %s", e.what());
    return 1;
  }

  // A dropped topic disappears entirely from the output: it is neither declared
  // nor carries any message. Every other topic is declared verbatim.
  std::size_t kept = 0;
  for (const auto & t : reader->topics()) {
    if (omit.count(t.name) != 0) {
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

  core::BagCopyCounts counts;
  try {
    counts = core::bag_copy_filtered(*reader, *writer, omit);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Stream copy from %s failed: %s", args.input_path.c_str(), e.what());
    return 1;
  }

  try {
    writer->close();
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Writer close() failed: %s", e.what());
    return 1;
  }

  BAGWIZ_LOG_INFO(
    kLogger,
    "topic omit: kept %zu topic(s), dropped %zu; copied %" PRIu64 " message(s), suppressed %" PRIu64
    ".",
    kept, omit.size(), counts.copied, counts.suppressed);
  return 0;
}

}  // namespace

int run_topic_omit(const TopicOmitArgs & args)
{
  // 0. Guard the public entry point. The CLI marks <topics> ->required(), but
  //    run_topic_omit is also called directly from tests; an empty selector
  //    list would otherwise slip past the unmatched-pattern check below and
  //    silently copy the bag unchanged. Fail fast instead.
  if (args.topics.empty()) {
    BAGWIZ_LOG_ERROR(kLogger, "topic omit: no topic selector given; nothing to remove.");
    return 1;
  }

  // 1. Resolve the selectors against the bag's topic list up front, so a typo'd
  //    or non-matching selector fails the run before any writer (or in-place
  //    tmp) is created. The topic list is snapshotted into a vector because the
  //    reader's span is invalidated once the reader is destroyed below.
  std::vector<std::string> topic_names;
  {
    std::unique_ptr<io::BagReader> reader;
    try {
      reader = io::open_read(args.input_path);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to open %s: %s", args.input_path.c_str(), e.what());
      return 1;
    }
    for (const auto & t : reader->topics()) {
      topic_names.push_back(t.name);
    }
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
  // ensure every selector matched at least one topic, so `omit` is non-empty.
  const auto & omit = resolution.matched;
  if (omit.size() == topic_names.size()) {
    BAGWIZ_LOG_WARN(
      kLogger, "all %zu topic(s) matched; the output bag will contain no topics.",
      topic_names.size());
  }

  // 2a. -o mode: write a new bag whose storage follows the output path (its
  //     extension picks a single-file backend; a directory inherits the input's
  //     backend) and leave <input> untouched.
  if (args.output_path.has_value()) {
    if (const auto r = core::prepare_output_path(*args.output_path, args.overwrite); !r.ok) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", r.error.c_str());
      return 1;
    }
    const auto input = args.input_path;
    const auto output = *args.output_path;
    auto make_writer = [input, output]() {
      auto copts = io::create_options_inheriting_format(input, output);
      copts.mcap_compression = "none";
      return io::open_write(output, copts);
    };
    return execute_omit_pass(args, omit, make_writer);
  }

  // 2b. In-place mode: rewrite <input> atomically via a sibling tmp, preserving
  //     its storage format and layout. The tmp path carries a synthetic suffix
  //     that Format::Auto cannot interpret, so pin both explicitly.
  const auto inplace_copts = io::create_options_preserving_storage(args.input_path);
  if (inplace_copts.format == io::Format::Auto) {
    BAGWIZ_LOG_ERROR(
      kLogger, "topic omit: could not detect storage format of input bag '%s'.",
      args.input_path.string().c_str());
    return 1;
  }
  auto make_inplace_writer = [inplace_copts](const std::filesystem::path & tmp) {
    auto copts = inplace_copts;
    copts.mcap_compression = "none";
    return io::open_write(tmp, copts);
  };

  // execute_omit_pass reports command-level failures via its return value
  // rather than throwing, so capture the status and translate a non-zero exit
  // into a runtime_error to make write_bag_inplace abort the swap (leaving
  // <input> untouched).
  int pass_status = 0;
  try {
    core::write_bag_inplace(args.input_path, [&](const std::filesystem::path & tmp) {
      pass_status = execute_omit_pass(args, omit, [&]() { return make_inplace_writer(tmp); });
      if (pass_status != 0) {
        throw std::runtime_error("topic omit: pass failed; aborting in-place swap");
      }
    });
  } catch (const std::exception & e) {
    // cppcheck-suppress knownConditionTrueFalse  // assigned inside the lambda above
    if (pass_status != 0) {
      return pass_status;  // the pass already logged the specific error
    }
    BAGWIZ_LOG_ERROR(kLogger, "In-place swap failed: %s", e.what());
    return 1;
  }
  return 0;
}

}  // namespace bagwiz::commands
