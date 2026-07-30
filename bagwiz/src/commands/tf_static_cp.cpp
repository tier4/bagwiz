// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/tf_static_cp.hpp"

#include "bagwiz/core/bag/bag_copy.hpp"
#include "bagwiz/core/bag/bag_topic_plan.hpp"
#include "bagwiz/core/bag/rewrite.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/tf/tf_message_wire.hpp"
#include "bagwiz/core/tf/tf_static_collect.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/bag_open.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace bagwiz::commands
{

namespace
{

constexpr const char * kLogger = "bagwiz.cmd.tf.static.cp";
constexpr const char * kTfMessageType = "tf2_msgs/msg/TFMessage";

// Set every transform's header.stamp to `stamp_ns` so the copied static TF
// carries the destination's start time rather than its source timestamps.
void restamp_transforms(
  std::vector<geometry_msgs::msg::TransformStamped> & transforms, std::int64_t stamp_ns)
{
  const auto sec = static_cast<std::int32_t>(stamp_ns / 1'000'000'000LL);
  const auto nanosec = static_cast<std::uint32_t>(stamp_ns % 1'000'000'000LL);
  for (auto & t : transforms) {
    t.header.stamp.sec = sec;
    t.header.stamp.nanosec = nanosec;
  }
}

// The plan for declaring/suppressing one source static topic against the
// destination bag.
struct TopicWritePlan
{
  std::unordered_set<std::string> suppress;     // copy-time drop set (replace)
  std::vector<std::string> declare_new_topics;  // topics absent from dst
};

// Decide, for every source static topic, whether it is declared fresh, kept,
// replaced (suppress + re-append), or aborts the run. `force` is the user's
// -w/--overwrite. Returns false (after logging) on an unresolved conflict.
bool plan_topic_writes(
  std::span<const io::TopicInfo> dst_topics,
  const std::unordered_map<std::string, std::int64_t> & dst_counts,
  const std::vector<core::StaticTopicTransforms> & src_topics, bool force,
  TopicWritePlan & plan_out)
{
  for (const auto & st : src_topics) {
    std::int64_t existing_count = 0;
    if (auto it = dst_counts.find(st.name); it != dst_counts.end()) {
      existing_count = it->second;
    }
    const auto decision =
      core::decide_topic_write(dst_topics, st.name, kTfMessageType, existing_count, force);
    switch (decision.action) {
      case core::TopicWriteAction::kConflictAbort:
      case core::TopicWriteAction::kTypeMismatch:
        BAGWIZ_LOG_ERROR(kLogger, "%s", decision.reason.c_str());
        return false;
      case core::TopicWriteAction::kDeclareAndSuppress:
        BAGWIZ_LOG_WARN(kLogger, "%s", decision.reason.c_str());
        plan_out.suppress.insert(st.name);
        break;
      case core::TopicWriteAction::kDeclareNew:
        plan_out.declare_new_topics.push_back(st.name);
        break;
      case core::TopicWriteAction::kDeclareKeep:
        // Topic exists but is empty: keep its declaration, just append.
        break;
    }
  }
  return true;
}

// One full copy pass: open the destination as the read side, plan topic
// conflicts, declare topics on the writer, stream-copy with suppression, then
// append one TFMessage per source static topic stamped at the destination's
// start time. Used for both in-place and explicit-output modes; the writer
// factory is parameterised so the rewrite dispatch (core::run_bag_rewrite)
// can hand in a tmp path.
int execute_cp_pass(
  const std::filesystem::path & dst_path,
  const std::vector<core::StaticTopicTransforms> & src_topics, bool overwrite,
  const io::WriterFactory & open_writer)
{
  auto reader = io::open_read_or_log(dst_path, kLogger);
  if (!reader) {
    return 1;
  }
  io::BagReader::TimeExtent time_extent;
  try {
    time_extent = reader->compute_time_extent();
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(
      kLogger, "Failed to compute time extent on %s: %s", dst_path.c_str(), e.what());
    return 1;
  }
  const std::int64_t start_ns = time_extent.start_ns;

  std::vector<std::string> src_topic_names;
  src_topic_names.reserve(src_topics.size());
  for (const auto & st : src_topics) {
    src_topic_names.push_back(st.name);
  }

  std::unordered_map<std::string, std::int64_t> dst_counts;
  try {
    dst_counts = reader->compute_topic_counts(src_topic_names);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(
      kLogger, "Failed to compute topic counts on %s: %s", dst_path.c_str(), e.what());
    return 1;
  }

  // Snapshot the destination's topic list before conflict detection; the
  // reader's span may be invalidated by subsequent operations.
  const std::vector<io::TopicInfo> dst_topics(reader->topics().begin(), reader->topics().end());

  TopicWritePlan plan;
  if (!plan_topic_writes(dst_topics, dst_counts, src_topics, overwrite, plan)) {
    return 1;
  }

  auto writer = io::open_write_or_log(open_writer, kLogger);
  if (!writer) {
    return 1;
  }

  // Schemas are only needed once we start streaming messages. Deferring
  // avoids opening shard 0 for bags that abort early due to a topic conflict.
  reader->populate_schemas();

  // Snapshot the destination's topic list after schema backfill so the output
  // writer receives embedded schemas.
  const std::vector<io::TopicInfo> dst_topics_with_schemas(
    reader->topics().begin(), reader->topics().end());

  // Declare every existing destination topic, then a synthesised TopicInfo for
  // each brand-new static topic being introduced.
  for (const auto & t : dst_topics_with_schemas) {
    try {
      writer->declare_topic(t);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "declare_topic failed for '%s': %s", t.name.c_str(), e.what());
      return 1;
    }
  }
  for (const auto & name : plan.declare_new_topics) {
    try {
      writer->declare_topic(core::make_tf_message_topic_info(name));
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(
        kLogger, "declare_topic failed for new topic '%s': %s", name.c_str(), e.what());
      return 1;
    }
  }

  // Emit the synthesized messages BEFORE the stream copy. They are stamped at
  // the destination's start time, so writing them afterwards would leave each
  // one holding the bag's lowest timestamp at the highest storage position —
  // the only rows whose physical order disagrees with their time. Consumers
  // that read a .db3 in row order rather than by timestamp (Foxglove's readers
  // issue their message query without an ORDER BY) would then receive the
  // static TF last, after everything it is supposed to precede.
  std::uint64_t injected = 0;
  core::TfMessageSerializer tf_serializer;
  for (const auto & st : src_topics) {
    auto transforms = st.transforms;
    restamp_transforms(transforms, start_ns);
    std::vector<std::byte> payload;
    try {
      tf_serializer.serialize_many(
        std::span<const geometry_msgs::msg::TransformStamped>(transforms.data(), transforms.size()),
        payload);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Failed to serialize TFMessage for topic '%s': %s", st.name.c_str(), e.what());
      return 1;
    }
    try {
      writer->write(st.name, start_ns, std::span<const std::byte>(payload.data(), payload.size()));
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Failed to write TFMessage on '%s' at stamp %" PRId64 ": %s", st.name.c_str(),
        start_ns, e.what());
      return 1;
    }
    ++injected;
  }

  core::BagCopyCounts counts;
  try {
    counts = core::bag_copy_filtered(
      *reader, *writer, plan.suppress, "tf_static_cp", core::pipeline::BackendKind::Pipelined);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Stream copy from %s failed: %s", dst_path.c_str(), e.what());
    return 1;
  }

  if (!io::close_writer_or_log(*writer, kLogger)) {
    return 1;
  }

  BAGWIZ_LOG_INFO(
    kLogger,
    "tf static cp: copied %" PRIu64 " message(s), suppressed %" PRIu64 ", injected %" PRIu64
    " static TF topic(s) at stamp %" PRId64 ".",
    counts.copied, counts.suppressed, injected, start_ns);
  return 0;
}

}  // namespace

int run_tf_static_cp(
  const std::filesystem::path & src_path, const std::filesystem::path & dst_path,
  const std::optional<std::filesystem::path> & output_path, bool overwrite)
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
  return core::run_bag_rewrite(
    dst_path, output_path, overwrite, rewrite_opts, [&](const io::WriterFactory & open_writer) {
      return execute_cp_pass(dst_path, src_topics, overwrite, open_writer);
    });
}

}  // namespace bagwiz::commands
