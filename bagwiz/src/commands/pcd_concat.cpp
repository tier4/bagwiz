// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/pcd_concat.hpp"

#include "bagwiz/core/bag/rewrite.hpp"
#include "bagwiz/core/base/duration_parse.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/pointcloud/cloud_transform.hpp"
#include "bagwiz/core/pointcloud/concat_sync.hpp"
#include "bagwiz/core/pointcloud/pointcloud2.hpp"
#include "bagwiz/core/pointcloud/static_extrinsic.hpp"
#include "bagwiz/core/tf/tf_buffer_loader.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/bag_open.hpp"
#include "bagwiz/io/topics.hpp"
#include "pcd_concat_common.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <tf2/buffer_core.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace bagwiz::commands
{
namespace
{
constexpr const char * kLogger = "bagwiz.cmd.pcd";
constexpr const char * kPointCloud2Type = "sensor_msgs/msg/PointCloud2";
constexpr std::chrono::hours kTfBufferCacheTime{24 * 365};
constexpr std::int64_t kDefaultToleranceNs = 50'000'000;  // 50 ms fallback
constexpr const char * kDefaultFrame = "base_link";       // --frame default

// geometry_msgs quaternion (x,y,z,w) + translation -> RigidTransform (row-major
// rotation matrix). p_target = R * p_source + t.
core::pointcloud::RigidTransform to_rigid(const geometry_msgs::msg::TransformStamped & ts)
{
  const double x = ts.transform.rotation.x;
  const double y = ts.transform.rotation.y;
  const double z = ts.transform.rotation.z;
  const double w = ts.transform.rotation.w;
  core::pointcloud::RigidTransform out;
  out.rotation = core::pointcloud::quat_to_rotation_matrix(x, y, z, w);
  out.translation = {
    ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z};
  return out;
}

std::int64_t median_period_ns(const std::vector<std::int64_t> & stamps)
{
  if (stamps.size() < 2) {
    return 0;
  }
  std::vector<std::int64_t> deltas;
  deltas.reserve(stamps.size() - 1);
  for (std::size_t i = 1; i < stamps.size(); ++i) {
    deltas.push_back(stamps[i] - stamps[i - 1]);
  }
  std::sort(deltas.begin(), deltas.end());
  return deltas[deltas.size() / 2];
}

// --pcd needs at least two distinct topics.
bool validate_pcd_topic_list(const PcdConcatArgs & args, const char * logger)
{
  if (args.pcd_topics.size() < 2) {
    BAGWIZ_LOG_ERROR(logger, "pcd concat: --pcd needs at least 2 topics");
    return false;
  }
  std::unordered_set<std::string> seen;
  for (const auto & t : args.pcd_topics) {
    if (!seen.insert(t).second) {
      BAGWIZ_LOG_ERROR(logger, "pcd concat: duplicate topic in --pcd: '%s'", t.c_str());
      return false;
    }
  }
  return true;
}

// Parse --tolerance (number + optional unit ns/us/ms/s, no unit = ms) when given.
bool parse_tolerance_override(
  const PcdConcatArgs & args, std::optional<std::int64_t> & tolerance_ns, const char * logger)
{
  if (!args.tolerance.has_value()) {
    return true;
  }
  const auto ns = core::parse_duration_ns(*args.tolerance);
  if (!ns.has_value() || *ns < 0) {
    BAGWIZ_LOG_ERROR(
      logger, "pcd concat: could not parse --tolerance '%s' (e.g. 50ms, 0.05s, 500us)",
      args.tolerance->c_str());
    return false;
  }
  tolerance_ns = *ns;
  return true;
}

// The explicit --tolerance, or half the reference topic's median period when it
// has one, or the 50 ms fallback.
std::int64_t effective_tolerance_ns(
  const std::optional<std::int64_t> & tolerance_override,
  const std::vector<std::int64_t> & ref_stamps_ns)
{
  if (tolerance_override.has_value()) {
    return *tolerance_override;
  }
  const std::int64_t period = median_period_ns(ref_stamps_ns);
  if (period > 0) {
    return period / 2;
  }
  return kDefaultToleranceNs;
}

// The resolved --pcd topics plus the output-topic collision probe.
struct ConcatTopics
{
  std::vector<const io::TopicInfo *> info_by_index;
  bool output_exists = false;
};

// Resolve every --pcd topic's TopicInfo (one lookup per topic) and check its
// type; then probe output-topic existence without logging on a miss. A
// pre-existing output topic requires --force.
std::optional<ConcatTopics> resolve_concat_topics(
  io::BagReader & reader, const PcdConcatArgs & args, const char * logger)
{
  ConcatTopics resolved;
  resolved.info_by_index.assign(args.pcd_topics.size(), nullptr);
  for (std::size_t i = 0; i < args.pcd_topics.size(); ++i) {
    const io::TopicInfo * info =
      io::find_topic_or_log(reader, args.pcd_topics[i], args.input_path, logger);
    if (info == nullptr) {
      return std::nullopt;
    }
    if (info->type != kPointCloud2Type) {
      BAGWIZ_LOG_ERROR(
        logger, "Topic '%s' is %s, expected %s", args.pcd_topics[i].c_str(), info->type.c_str(),
        kPointCloud2Type);
      return std::nullopt;
    }
    resolved.info_by_index[i] = info;
  }
  resolved.output_exists = io::find_topic(reader, args.output_topic) != nullptr;
  if (resolved.output_exists && !args.force) {
    BAGWIZ_LOG_ERROR(
      logger, "Output topic '%s' already exists in %s; pass --force to replace it",
      args.output_topic.c_str(), args.input_path.c_str());
    return std::nullopt;
  }
  return resolved;
}

// Pass A: stream the input once (filtered to the --pcd topics) to collect each
// topic's header stamps, first frame_id, and the Pass-A diagnostics. Every
// topic must yield at least one stamp and a frame_id.
std::optional<std::vector<TopicState>> collect_topic_stamps(
  const std::filesystem::path & input_path, const std::vector<std::string> & pcd_topics,
  const std::vector<std::int64_t> & offsets,
  const std::unordered_map<std::string, std::size_t> & topic_index, const char * logger)
{
  std::vector<TopicState> topics(pcd_topics.size());
  for (std::size_t i = 0; i < pcd_topics.size(); ++i) {
    topics[i].name = pcd_topics[i];
    topics[i].offset_ns = offsets[i];
  }

  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(input_path);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(logger, "Failed to reopen %s: %s", input_path.c_str(), e.what());
    return std::nullopt;
  }
  io::ReadFilter filter;
  filter.topics = pcd_topics;
  reader->set_filter(filter);
  io::RawMessage raw;
  try {
    while (reader->next(raw)) {
      const auto it = topic_index.find(raw.topic->name);
      if (it == topic_index.end()) {
        continue;
      }
      TopicState & ts = topics[it->second];
      const auto header = core::pointcloud::parse_pointcloud2_header(raw.payload);
      if (header.ok()) {
        if (!ts.stamps_ns.empty() && header.header->timestamp_ns < ts.stamps_ns.back()) {
          ts.non_monotonic = true;
        }
        ts.stamps_ns.push_back(header.header->timestamp_ns);
        if (ts.frame_id.empty()) {
          ts.frame_id = header.header->frame_id;
        }
      } else {
        // keep index alignment with Pass B by recording the bag stamp
        ts.stamps_ns.push_back(raw.timestamp_ns);
        ++ts.header_fail;
      }
    }
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(logger, "read error collecting stamps: %s", e.what());
    return std::nullopt;
  }

  for (const auto & ts : topics) {
    if (ts.stamps_ns.empty()) {
      BAGWIZ_LOG_ERROR(logger, "Topic '%s' has no decodable PointCloud2 messages", ts.name.c_str());
      return std::nullopt;
    }
    if (ts.frame_id.empty()) {
      BAGWIZ_LOG_ERROR(logger, "Could not read a frame_id from '%s'", ts.name.c_str());
      return std::nullopt;
    }
  }
  return topics;
}

// Resolve each topic's extrinsic (target_frame <- frame_id) from the bag's
// static TF; topics already in the target frame get the identity.
bool resolve_extrinsics(
  std::vector<TopicState> & topics, const std::filesystem::path & input_path,
  const std::string & target_frame, bool frame_explicit, const char * logger)
{
  tf2::BufferCore buffer{kTfBufferCacheTime};
  bool need_tf = false;
  for (const auto & ts : topics) {
    if (ts.frame_id != target_frame) {
      need_tf = true;
    }
  }
  if (need_tf) {
    if (const auto error = core::load_static_tf_buffer(input_path, buffer); error.has_value()) {
      // load_static_tf_buffer is a shared, caller-neutral helper (it names
      // no command's flags); pcd concat owns --frame, so it supplies that
      // context itself here.
      BAGWIZ_LOG_ERROR(
        logger, "pcd concat: cannot resolve input topic extrinsics to --frame '%s': %s",
        target_frame.c_str(), error->c_str());
      return false;
    }
  }
  for (auto & ts : topics) {
    if (ts.frame_id == target_frame) {
      ts.extrinsic = core::pointcloud::RigidTransform{};  // identity
      continue;
    }
    // Resolve target_frame <- ts.frame_id from the static TF. If --frame was
    // not given and a topic cannot reach the default (base_link), the default
    // does not span every input, so --frame is required.
    std::string reach_error;
    const auto resolved =
      core::pointcloud::resolve_static_extrinsic(buffer, target_frame, ts.frame_id);
    if (!resolved.missing.empty()) {
      std::string names;
      for (std::size_t i = 0; i < resolved.missing.size(); ++i) {
        names += (i ? ", " : "") + resolved.missing[i];
      }
      reach_error = "frame(s) not present in the bag's static TF tree: " + names;
    } else if (!resolved.ok()) {
      reach_error = "no static TF chain from '" + target_frame + "' to '" + ts.frame_id +
                    "': " + resolved.lookup_error;
    } else {
      ts.extrinsic = to_rigid(resolved.transform);
    }
    if (!reach_error.empty()) {
      if (frame_explicit) {
        BAGWIZ_LOG_ERROR(logger, "%s", reach_error.c_str());
      } else {
        BAGWIZ_LOG_ERROR(
          logger,
          "pcd concat: --frame is required — the default frame '%s' is not reachable from '%s' "
          "(frame '%s'); pass --frame explicitly [%s]",
          target_frame.c_str(), ts.name.c_str(), ts.frame_id.c_str(), reach_error.c_str());
      }
      return false;
    }
  }
  return true;
}

// The plan_sync view of the resolved topics: header stamps + offset per topic.
std::vector<core::pointcloud::SyncTopic> to_sync_topics(const std::vector<TopicState> & topics)
{
  std::vector<core::pointcloud::SyncTopic> sync_topics(topics.size());
  for (std::size_t i = 0; i < topics.size(); ++i) {
    sync_topics[i].stamps_ns = topics[i].stamps_ns;
    sync_topics[i].offset_ns = topics[i].offset_ns;
  }
  return sync_topics;
}

// Topics suppressed from copy-through: dropped pcd inputs, and a pre-existing
// output topic being replaced (--force).
std::unordered_set<std::string> build_suppress_set(const PcdConcatArgs & args, bool output_exists)
{
  std::unordered_set<std::string> suppress;
  if (args.drop_inputs) {
    for (const auto & t : args.pcd_topics) {
      suppress.insert(t);
    }
  }
  if (output_exists) {
    suppress.insert(args.output_topic);
  }
  return suppress;
}

// topic name -> --pcd index.
std::unordered_map<std::string, std::size_t> topic_index_by_name(
  const std::vector<std::string> & pcd_topics)
{
  std::unordered_map<std::string, std::size_t> index;
  for (std::size_t i = 0; i < pcd_topics.size(); ++i) {
    index[pcd_topics[i]] = i;
  }
  return index;
}

// The bag-rewrite options pcd concat always uses.
core::BagRewriteOptions pcd_concat_rewrite_options(const char * logger)
{
  core::BagRewriteOptions opts;
  opts.logger = logger;
  opts.format_unknown_error = "pcd concat: could not detect storage format of input bag '%s'.";
  opts.pass_failed_error = "pcd concat: pass failed; aborting in-place swap";
  opts.inherit_output_format = true;
  return opts;
}

// Pass B: open the writer, declare the surviving topics, stream the input
// again, copy the unsuppressed messages through, feed the pcd inputs to the
// assembler, and write each fired group's merged cloud. Returns an exit code.
int execute_concat_pass(
  const io::WriterFactory & factory, const io::BagReader & reader, const PcdConcatArgs & args,
  const std::unordered_map<std::string, std::size_t> & topic_index,
  const std::unordered_set<std::string> & suppress, const io::TopicInfo & out_topic,
  ConcatAssembler & assembler, const char * logger)
{
  auto writer = io::open_write_or_log(factory, logger);
  if (!writer) {
    return 1;
  }

  // declare surviving input topics + the new output topic
  for (const auto & t : reader.topics()) {
    if (suppress.count(t.name) != 0 || t.name == args.output_topic) {
      continue;
    }
    writer->declare_topic(t);
  }
  writer->declare_topic(out_topic);

  std::unique_ptr<io::BagReader> rd;
  try {
    rd = io::open_read(args.input_path);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(logger, "Failed to reopen %s: %s", args.input_path.c_str(), e.what());
    return 1;
  }
  rd->populate_schemas();

  std::vector<std::size_t> seen(topic_index.size(), 0);
  io::RawMessage raw;
  while (rd->next(raw)) {
    const std::string & name = raw.topic->name;
    const auto ti = topic_index.find(name);

    // copy-through unless suppressed
    if (suppress.count(name) == 0 && name != args.output_topic) {
      writer->write(name, raw.timestamp_ns, raw.payload);
    }

    if (ti == topic_index.end()) {
      continue;  // not a pcd input topic
    }
    const std::size_t t = ti->second;
    const std::size_t idx = seen[t]++;

    auto result = assembler.on_message(t, idx, raw.payload);
    for (const auto & output : result.fired) {
      writer->write(args.output_topic, output.stamp_ns, output.payload);
    }
    if (!result.error.empty()) {
      BAGWIZ_LOG_ERROR(logger, "concat failed: %s", result.error.c_str());
      return 1;
    }
  }

  if (!io::close_writer_or_log(*writer, logger)) {
    return 1;
  }
  return 0;
}

// The end-of-run summary: one INFO line for the run, one per topic, then a WARN
// line per topic with failures or non-monotonic stamps.
void log_concat_summary(
  const std::vector<TopicState> & topics, const ConcatAssembler::Counters & counters,
  std::int64_t tolerance_ns, const std::string & output_topic, const char * logger)
{
  BAGWIZ_LOG_INFO(
    logger,
    "pcd concat: wrote %" PRId64 " concatenated message(s) to '%s' (%" PRId64
    " partial, tolerance %.3f ms)",
    counters.written_groups, output_topic.c_str(), counters.partial_groups,
    static_cast<double>(tolerance_ns) / 1e6);
  for (std::size_t i = 0; i < topics.size(); ++i) {
    BAGWIZ_LOG_INFO(
      logger, "  %s: matched %" PRId64 " (frame '%s', offset %.3f ms)", topics[i].name.c_str(),
      counters.matched[i], topics[i].frame_id.c_str(),
      static_cast<double>(topics[i].offset_ns) / 1e6);
  }
  for (std::size_t i = 0; i < topics.size(); ++i) {
    if (
      topics[i].header_fail != 0 || counters.parse_fail[i] != 0 ||
      counters.transform_fail[i] != 0 || topics[i].non_monotonic) {
      BAGWIZ_LOG_WARN(
        logger,
        "  %s: %" PRId64 " undecodable header(s) [bag time used for matching], %" PRId64
        " parse + %" PRId64 " transform failure(s) dropped from concat%s",
        topics[i].name.c_str(), topics[i].header_fail, counters.parse_fail[i],
        counters.transform_fail[i],
        topics[i].non_monotonic ? "; header stamps are not monotonic (matching may be wrong)" : "");
    }
  }
}

}  // namespace

int run_pcd_concat(const PcdConcatArgs & args)
{
  // ---- validate arguments -------------------------------------------------
  if (!validate_pcd_topic_list(args, kLogger)) {
    return 1;
  }
  // --frame defaults to base_link; when the default cannot reach every --pcd
  // frame via static TF, --frame becomes required (extrinsic resolution below).
  const bool frame_explicit = args.frame.has_value();
  const std::string target_frame = frame_explicit ? *args.frame : std::string(kDefaultFrame);
  const std::size_t ref_idx = 0;  // the first --pcd topic drives output rate + stamps
  const auto topic_index = topic_index_by_name(args.pcd_topics);
  const auto offsets = parse_stamp_offsets(args.stamp_offsets, topic_index, kLogger);
  if (!offsets.has_value()) {
    return 1;
  }
  std::optional<std::int64_t> tolerance_override;
  if (!parse_tolerance_override(args, tolerance_override, kLogger)) {
    return 1;
  }

  // ---- open reader, validate topics ---------------------------------------
  auto reader = io::open_read_or_log(args.input_path, kLogger);
  if (!reader) {
    return 1;
  }
  reader->populate_schemas();
  const auto resolved = resolve_concat_topics(*reader, args, kLogger);
  if (!resolved.has_value()) {
    return 1;
  }

  // ---- Pass A (collect stamps), then extrinsics (target = --frame) ---------
  auto topics =
    collect_topic_stamps(args.input_path, args.pcd_topics, *offsets, topic_index, kLogger);
  if (!topics.has_value()) {
    return 1;
  }
  if (!resolve_extrinsics(*topics, args.input_path, target_frame, frame_explicit, kLogger)) {
    return 1;
  }

  // ---- tolerance + sync plan + streaming pass (-o vs in-place) -------------
  const std::int64_t tolerance_ns =
    effective_tolerance_ns(tolerance_override, (*topics)[ref_idx].stamps_ns);
  ConcatAssembler assembler(
    *topics, core::pointcloud::plan_sync(to_sync_topics(*topics), ref_idx, tolerance_ns),
    target_frame);
  const std::unordered_set<std::string> suppress =
    build_suppress_set(args, resolved->output_exists);
  io::TopicInfo out_topic = *resolved->info_by_index[ref_idx];
  out_topic.name = args.output_topic;
  const int status = core::run_bag_rewrite(
    args.input_path, args.output_path, args.overwrite, pcd_concat_rewrite_options(kLogger),
    [&](const io::WriterFactory & factory) {
      return execute_concat_pass(
        factory, *reader, args, topic_index, suppress, out_topic, assembler, kLogger);
    });
  if (status != 0) {
    return status;
  }

  log_concat_summary(*topics, assembler.counters(), tolerance_ns, args.output_topic, kLogger);
  return 0;
}

}  // namespace bagwiz::commands
