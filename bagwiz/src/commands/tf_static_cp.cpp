// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/tf_static_cp.hpp"

#include "bagwiz/core/bag/rewrite.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/tf/tf_static_collect.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "tf_static_inject.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <exception>
#include <filesystem>
#include <optional>
#include <vector>

namespace bagwiz::commands
{

namespace
{

constexpr const char * kLogger = "bagwiz.cmd.tf.static.cp";

}  // namespace

int run_tf_static_cp(
  const std::filesystem::path & src_path, const std::filesystem::path & dst_path,
  const std::optional<std::filesystem::path> & output_path, bool force, bool overwrite)
{
  // 1. Gather the static TF topics + transforms from the source bag.
  std::vector<core::StaticTopicTransforms> src_topics;
  try {
    // Whole-topic: a copy must carry every edge the source declares, including
    // one a second broadcaster only added on a later message.
    src_topics = core::collect_static_tf(src_path, core::StaticTfRead::kWholeTopic);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Failed to read static TF from %s: %s", src_path.c_str(), e.what());
    return 1;
  }
  if (src_topics.empty()) {
    BAGWIZ_LOG_ERROR(
      kLogger,
      "Source bag %s has no static tf2_msgs/msg/TFMessage topic (e.g. /tf_static) carrying "
      "transforms; nothing to copy.",
      src_path.c_str());
    return 1;
  }

  // 2. -o vs in-place dispatch, shared with the other rewrite-style commands:
  //    -o writes a fresh bag (format/layout resolved from the output path's
  //    extension) and leaves <dst> untouched; otherwise <dst> is rewritten
  //    atomically via a sibling tmp, preserving its storage identity. The
  //    dispatch rewrites the destination bag, so it is passed as the input.
  core::BagRewriteOptions rewrite_opts;
  rewrite_opts.logger = kLogger;
  rewrite_opts.format_unknown_error =
    "tf static cp: could not detect storage format of destination bag '%s'.";
  rewrite_opts.pass_failed_error = "tf static cp: pass failed; aborting in-place swap";
  StaticTfInjectOptions inject_opts;
  inject_opts.logger = kLogger;
  inject_opts.label = "tf static cp";
  inject_opts.profile_label = "tf_static_cp";
  // --force covers replacing a colliding topic in <dst>; -w/--overwrite covers only
  // the -o path (handled by run_bag_rewrite below).
  inject_opts.replace_existing_topic = force;
  return core::run_bag_rewrite(
    dst_path, output_path, overwrite, rewrite_opts, [&](const io::WriterFactory & open_writer) {
      return inject_static_tf_pass(dst_path, src_topics, inject_opts, open_writer);
    });
}

}  // namespace bagwiz::commands
