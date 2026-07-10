// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/tf_static_cp.hpp"

#include "bagwiz/core/bag_copy.hpp"
#include "bagwiz/core/bag_inplace.hpp"
#include "bagwiz/core/bag_topic_plan.hpp"
#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/logging.hpp"
#include "bagwiz/core/output_path.hpp"
#include "bagwiz/core/tf_message_wire.hpp"
#include "bagwiz/core/tf_value_extract.hpp"
#include "bagwiz/io/bag_io.hpp"

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
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{

constexpr const char * kLogger = "bagwiz.cmd.tf.static.cp";
constexpr const char * kTfMessageType = "tf2_msgs/msg/TFMessage";
constexpr std::string_view kTfStaticSuffix = "tf_static";

// True when a TF topic's name marks it static (ends with "tf_static", e.g.
// "/tf_static"). `tf static cp` copies only these topics.
bool is_static_tf_topic(std::string_view topic_name)
{
  if (topic_name.size() < kTfStaticSuffix.size()) {
    return false;
  }
  return topic_name.compare(
           topic_name.size() - kTfStaticSuffix.size(), kTfStaticSuffix.size(), kTfStaticSuffix) ==
         0;
}

// The transforms gathered from one source static TF topic, ready to be written
// into the destination under the same name. `transforms` is deduplicated by
// child_frame_id (last value wins) so a republished static topic collapses to
// the single latched set it represents; first-seen order is preserved.
struct StaticTopicTransforms
{
  std::string name;
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
};

// Open one decoder per static TF topic in `reader`. Throws std::runtime_error
// if a decoder cannot be constructed (no embedded schema and no typesupport).
std::unordered_map<std::string, std::unique_ptr<core::decoder::Decoder>> open_static_tf_decoders(
  const io::BagReader & reader)
{
  std::unordered_map<std::string, std::unique_ptr<core::decoder::Decoder>> decoder_by_topic;
  for (const auto & topic_info : reader.topics()) {
    if (topic_info.type != kTfMessageType || !is_static_tf_topic(topic_info.name)) {
      continue;
    }
    auto open = core::decoder::open_decoder(topic_info);
    if (!open.ok()) {
      throw std::runtime_error(
        "Could not open decoder for static TF topic '" + topic_info.name + "': " + open.error);
    }
    decoder_by_topic.emplace(topic_info.name, std::move(open.decoder));
  }
  return decoder_by_topic;
}

// Merge one message's transforms into a topic's accumulator: a child_frame_id
// already seen is overwritten in place (last wins) so a re-published static
// topic collapses to its latched set; a new one is appended (first-seen order).
void merge_transforms(
  std::vector<geometry_msgs::msg::TransformStamped> & transforms,
  std::unordered_map<std::string, std::size_t> & child_index,
  const std::vector<geometry_msgs::msg::TransformStamped> & incoming)
{
  for (const auto & t : incoming) {
    const auto ins = child_index.emplace(t.child_frame_id, transforms.size());
    if (ins.second) {
      transforms.push_back(t);
    } else {
      transforms[ins.first->second] = t;
    }
  }
}

// Read every static TF topic from `src_path` and return the deduplicated
// transforms per topic, in the order the topics appear in the bag. Topics that
// yield no transforms are dropped. Throws std::runtime_error on open / decoder
// / decode failure.
std::vector<StaticTopicTransforms> collect_src_static_tf(const std::filesystem::path & src_path)
{
  auto reader = io::open_read(src_path);
  reader->populate_schemas();

  std::vector<std::string> static_topics;
  for (const auto & t : reader->topics()) {
    if (t.type == kTfMessageType && is_static_tf_topic(t.name)) {
      static_topics.push_back(t.name);
    }
  }
  if (static_topics.empty()) {
    return {};
  }

  auto decoder_by_topic = open_static_tf_decoders(*reader);

  io::ReadFilter filter;
  filter.topics = static_topics;
  reader->set_filter(filter);

  // Per topic: the accumulating transform list plus a child_frame_id -> index
  // map (see merge_transforms) keyed by topic name.
  std::unordered_map<std::string, std::vector<geometry_msgs::msg::TransformStamped>> by_topic;
  std::unordered_map<std::string, std::unordered_map<std::string, std::size_t>>
    child_index_by_topic;

  io::RawMessage raw;
  while (reader->next(raw)) {
    auto it = decoder_by_topic.find(raw.topic->name);
    if (it == decoder_by_topic.end()) {
      continue;
    }
    const auto decoded = it->second->decode(raw.payload);
    if (!decoded.ok()) {
      throw std::runtime_error(
        "Failed to decode static TF message on '" + raw.topic->name + "': " + decoded.error);
    }
    merge_transforms(
      by_topic[raw.topic->name], child_index_by_topic[raw.topic->name],
      core::extract_tf_message(*decoded.value));
  }

  std::vector<StaticTopicTransforms> out;
  for (const auto & name : static_topics) {
    auto found = by_topic.find(name);
    if (found == by_topic.end() || found->second.empty()) {
      continue;
    }
    out.push_back({name, std::move(found->second)});
  }
  return out;
}

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
  const std::vector<StaticTopicTransforms> & src_topics, bool force, TopicWritePlan & plan_out)
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
// factory is parameterised so write_bag_inplace can hand in a tmp path.
int execute_cp_pass(
  const std::filesystem::path & dst_path, const std::vector<StaticTopicTransforms> & src_topics,
  bool overwrite, const std::function<std::unique_ptr<io::BagWriter>()> & open_writer)
{
  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(dst_path);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Failed to open %s: %s", dst_path.c_str(), e.what());
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

  std::unique_ptr<io::BagWriter> writer;
  try {
    writer = open_writer();
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Failed to open output writer: %s", e.what());
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

  core::BagCopyCounts counts;
  try {
    counts = core::bag_copy_filtered(
      *reader, *writer, plan.suppress, "tf_static_cp", core::pipeline::BackendKind::Pipelined);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Stream copy from %s failed: %s", dst_path.c_str(), e.what());
    return 1;
  }

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

  try {
    writer->close();
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Writer close() failed: %s", e.what());
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
  std::vector<StaticTopicTransforms> src_topics;
  try {
    src_topics = collect_src_static_tf(src_path);
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

  // 2. Explicit -o: write a fresh bag, leaving <dst> untouched. The writer
  //    picks format/layout from the output path's extension (.mcap / .db3 /
  //    directory).
  if (output_path.has_value()) {
    if (const auto r = core::prepare_output_path(*output_path, overwrite); !r.ok) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", r.error.c_str());
      return 1;
    }
    auto make_writer = [&]() {
      io::CreateOptions copts;
      copts.format = io::Format::Auto;
      copts.layout = io::Layout::Auto;
      copts.mcap_compression = "none";
      return io::open_write(*output_path, copts);
    };
    return execute_cp_pass(dst_path, src_topics, overwrite, make_writer);
  }

  // 3. In-place mode: pin format/layout to <dst>'s identity. The tmp path used
  //    by write_bag_inplace carries a synthetic suffix that Format::Auto /
  //    Layout::Auto cannot interpret, so preserve them explicitly.
  const auto inplace_copts = io::create_options_preserving_storage(dst_path);
  if (inplace_copts.format == io::Format::Auto) {
    BAGWIZ_LOG_ERROR(
      kLogger, "tf static cp: could not detect storage format of destination bag '%s'.",
      dst_path.string().c_str());
    return 1;
  }
  auto make_inplace_writer = [inplace_copts](const std::filesystem::path & tmp) {
    auto copts = inplace_copts;
    copts.mcap_compression = "none";
    return io::open_write(tmp, copts);
  };

  // write_bag_inplace materialises the tmp and swaps on success. execute_cp_pass
  // returns int rather than throwing for command-level errors, so surface a
  // non-zero exit via a captured status and translate it into an exception so
  // the in-place helper aborts the swap and leaves <dst> untouched.
  int pass_status = 0;
  try {
    core::write_bag_inplace(dst_path, [&](const std::filesystem::path & tmp) {
      pass_status = execute_cp_pass(
        dst_path, src_topics, overwrite, [&]() { return make_inplace_writer(tmp); });
      if (pass_status != 0) {
        throw std::runtime_error("tf static cp: pass failed; aborting in-place swap");
      }
    });
  } catch (const std::exception & e) {
    // cppcheck-suppress knownConditionTrueFalse  // assigned inside the lambda above
    if (pass_status != 0) {
      // The pass already logged the specific error.
      return pass_status;
    }
    BAGWIZ_LOG_ERROR(kLogger, "In-place swap failed: %s", e.what());
    return 1;
  }
  return 0;
}

}  // namespace bagwiz::commands
