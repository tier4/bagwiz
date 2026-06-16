// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/topic_rename.hpp"

#include "bagwiz/core/bag_copy.hpp"
#include "bagwiz/core/bag_inplace.hpp"
#include "bagwiz/core/logging.hpp"
#include "bagwiz/core/output_path.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <algorithm>
#include <cinttypes>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <functional>
#include <memory>
#include <stdexcept>
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
// modes; the writer factory is injected so write_bag_inplace can supply a tmp
// path. Returns a process exit code.
int execute_rename_pass(
  const TopicRenameArgs & args, const std::function<std::unique_ptr<io::BagWriter>()> & open_writer)
{
  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(args.input_path);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Failed to open %s: %s", args.input_path.c_str(), e.what());
    return 1;
  }
  // Backfill embedded schemas so MCAP outputs keep self-description for the
  // renamed topic (no-op for single-file readers and SQLite3 inputs). The
  // schema is keyed by message type, not topic name, so it survives the rename
  // unchanged.
  reader->populate_schemas();

  std::unique_ptr<io::BagWriter> writer;
  try {
    writer = open_writer();
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Failed to open output writer: %s", e.what());
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
    counts = core::bag_copy_renamed(*reader, *writer, rename);
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
    return execute_rename_pass(args, make_writer);
  }

  // 2b. In-place mode: rewrite <input> atomically via a sibling tmp, preserving
  //     its storage format and layout. The tmp path carries a synthetic suffix
  //     that Format::Auto cannot interpret, so pin both explicitly.
  const auto inplace_copts = io::create_options_preserving_storage(args.input_path);
  if (inplace_copts.format == io::Format::Auto) {
    BAGWIZ_LOG_ERROR(
      kLogger, "topic rename: could not detect storage format of input bag '%s'.",
      args.input_path.string().c_str());
    return 1;
  }
  auto make_inplace_writer = [inplace_copts](const std::filesystem::path & tmp) {
    auto copts = inplace_copts;
    copts.mcap_compression = "none";
    return io::open_write(tmp, copts);
  };

  // execute_rename_pass reports command-level failures via its return value
  // rather than throwing, so capture the status and translate a non-zero exit
  // into a runtime_error to make write_bag_inplace abort the swap (leaving
  // <input> untouched).
  int pass_status = 0;
  try {
    core::write_bag_inplace(args.input_path, [&](const std::filesystem::path & tmp) {
      pass_status = execute_rename_pass(args, [&]() { return make_inplace_writer(tmp); });
      if (pass_status != 0) {
        throw std::runtime_error("topic rename: pass failed; aborting in-place swap");
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
