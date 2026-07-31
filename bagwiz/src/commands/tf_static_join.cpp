// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/tf_static_join.hpp"

#include "bagwiz/core/bag/rewrite.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/base/str_utils.hpp"
#include "bagwiz/core/tf/tf_static_collect.hpp"
#include "bagwiz/core/tf/tf_static_tree_yaml.hpp"
#include "bagwiz/core/tf/tf_topics.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "tf_static_inject.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <cinttypes>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{

constexpr const char * kLogger = "bagwiz.cmd.tf.static.join";

}  // namespace

int run_tf_static_join(
  const std::filesystem::path & input_path, const std::filesystem::path & yaml_path,
  const std::string & topic, const std::optional<std::filesystem::path> & output_path, bool force,
  bool overwrite)
{
  // CLI11 marks -t/--topic as having a default but still accepts an explicit
  // empty string, which would declare an unnameable topic.
  if (topic.empty()) {
    BAGWIZ_LOG_ERROR(kLogger, "-t/--topic must be a non-empty topic name.");
    return 1;
  }
  // Not an error: a caller may have their own reader for a differently-named
  // latched topic. But bagwiz's own static-TF readers (`tf static dump`,
  // `tf static calc`, `tf tree -t static`, `tf static cp`) all select topics by
  // the "tf_static" name suffix, so they would not see this one.
  if (!core::is_static_tf_topic(topic)) {
    BAGWIZ_LOG_WARN(
      kLogger,
      "Topic '%s' does not end with 'tf_static', so bagwiz's static-TF readers will treat it as a "
      "dynamic TF topic and skip it.",
      topic.c_str());
  }

  // 1. Read the publisher config. Strict by design: this is a hand-edited file,
  //    where a silently-ignored key becomes a silently-wrong sensor pose.
  const auto parsed = core::parse_static_tf_tree_yaml(yaml_path);
  if (!parsed.ok()) {
    BAGWIZ_LOG_ERROR(
      kLogger, "Could not load static TF from '%s': %s", yaml_path.c_str(), parsed.error.c_str());
    return 1;
  }
  // Nesting deeper than two levels is a grouping heading, not a chain: only the
  // level immediately above a transform names its parent. That is legal and
  // lossless, but an author who wrote `a: {b: {c: {x: ...}}}` expecting a -> b -> c
  // gets only b -> c, so name the keys that turned out to parent nothing.
  if (!parsed.grouping_frames.empty()) {
    BAGWIZ_LOG_WARN(
      kLogger,
      "'%s': %s named a grouping level, not a parent frame, so no transform was created for it. "
      "Only the level directly above a transform is its parent.",
      yaml_path.c_str(), core::join_csv(parsed.grouping_frames).c_str());
  }
  std::vector<core::StaticTopicTransforms> topics;
  topics.push_back({topic, *parsed.transforms});
  const auto transform_count = static_cast<std::uint64_t>(topics.front().transforms.size());

  // 2. -o vs in-place dispatch, shared with the other rewrite-style commands:
  //    -o writes a fresh bag (format/layout resolved from the output path's
  //    extension) and leaves <input> untouched; otherwise <input> is rewritten
  //    atomically via a sibling tmp, preserving its storage identity.
  core::BagRewriteOptions rewrite_opts;
  rewrite_opts.logger = kLogger;
  rewrite_opts.format_unknown_error =
    "tf static join: could not detect storage format of input bag '%s'.";
  rewrite_opts.pass_failed_error = "tf static join: pass failed; aborting in-place swap";

  StaticTfInjectOptions inject_opts;
  inject_opts.logger = kLogger;
  inject_opts.label = "tf static join";
  inject_opts.profile_label = "tf_static_join";
  // Unlike `cp`, `join` splits the two conflicts (matching `traj join`): --force
  // covers replacing an existing topic, -w/--overwrite only the -o path.
  inject_opts.replace_existing_topic = force;

  const int rc = core::run_bag_rewrite(
    input_path, output_path, overwrite, rewrite_opts, [&](const io::WriterFactory & open_writer) {
      return inject_static_tf_pass(input_path, topics, inject_opts, open_writer);
    });
  if (rc == 0) {
    BAGWIZ_LOG_INFO(
      kLogger, "tf static join: embedded %" PRIu64 " transform(s) from '%s' on '%s'.",
      transform_count, yaml_path.string().c_str(), topic.c_str());
  }
  return rc;
}

}  // namespace bagwiz::commands
