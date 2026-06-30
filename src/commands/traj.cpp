// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/core/bag_copy.hpp"
#include "bagwiz/core/bag_inplace.hpp"
#include "bagwiz/core/bag_topic_plan.hpp"
#include "bagwiz/core/cdr_walker/value.hpp"
#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/logging.hpp"
#include "bagwiz/core/output_path.hpp"
#include "bagwiz/core/tf_chain.hpp"
#include "bagwiz/core/tf_merge_check.hpp"
#include "bagwiz/core/tf_message_wire.hpp"
#include "bagwiz/core/tf_value_extract.hpp"
#include "bagwiz/core/trajectory.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2/buffer_core.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <fmt/core.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
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

constexpr const char * kLogger = "bagwiz.cmd.traj";
constexpr const char * kFormatTum = "tum";
constexpr const char * kJoinMsgTypeTf = "tf";
constexpr const char * kTfMessageType = "tf2_msgs/msg/TFMessage";
constexpr const char * kPoseStampedType = "geometry_msgs/msg/PoseStamped";
constexpr const char * kPoseWithCovarianceStampedType =
  "geometry_msgs/msg/PoseWithCovarianceStamped";
constexpr const char * kOdometryType = "nav_msgs/msg/Odometry";

enum class PoseDumpKind { PoseStamped, PoseWithCovarianceStamped, Odometry };
constexpr std::string_view kTfStaticSuffix = "tf_static";

// /tf_static and any topic whose name terminates in "tf_static" use the
// transient_local durability and carry one-shot, time-independent
// transforms. Everything else carrying TFMessage is treated as dynamic
// and stored in the time-indexed history.
bool is_static_tf_topic(std::string_view topic_name)
{
  if (topic_name.size() < kTfStaticSuffix.size()) {
    return false;
  }
  return topic_name.compare(
           topic_name.size() - kTfStaticSuffix.size(), kTfStaticSuffix.size(), kTfStaticSuffix) ==
         0;
}

// Lowercase extension without leading dot, or empty when missing / not usable.
std::string output_path_extension_lower(const std::filesystem::path & output_path)
{
  std::string ext = output_path.extension().string();
  if (ext.size() <= 1U) {
    return {};
  }
  ext.erase(0, 1);
  for (char & c : ext) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return ext;
}

// When `format_opt` is non-empty, it wins (must be a supported id). When empty,
// `output_path`'s extension selects the format (e.g. ".tum" -> tum).
bool resolve_dump_format(
  const std::string & format_opt, const std::filesystem::path & output_path, std::string & out)
{
  if (!format_opt.empty()) {
    if (format_opt != kFormatTum) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Unsupported output format '%s'. Supported: %s.", format_opt.c_str(), kFormatTum);
      return false;
    }
    out = format_opt;
    return true;
  }

  const std::string ext = output_path_extension_lower(output_path);
  if (ext.empty()) {
    BAGWIZ_LOG_ERROR(
      kLogger,
      "Output format is not set (-f/--format) and the output path '%s' has no usable extension; "
      "use e.g. '*.tum' or pass --format %s.",
      output_path.c_str(), kFormatTum);
    return false;
  }
  if (ext == kFormatTum) {
    out = kFormatTum;
    return true;
  }

  BAGWIZ_LOG_ERROR(
    kLogger,
    "Output format is not set (-f/--format) and extension '.%s' is not recognized; "
    "use '*.tum' or pass --format %s.",
    ext.c_str(), kFormatTum);
  return false;
}

struct TfTopic
{
  std::string name;
  bool is_static = false;
};

std::vector<TfTopic> collect_tf_topics(const io::BagReader & reader)
{
  std::vector<TfTopic> topics;
  for (const auto & t : reader.topics()) {
    if (t.type == kTfMessageType) {
      topics.push_back({t.name, is_static_tf_topic(t.name)});
    }
  }
  return topics;
}

// One observed edge from the input topic. Stored separately from the
// TF buffer so we can filter by chain-edge membership after the chain
// has been resolved.
struct InputEdge
{
  std::string frame_id;
  std::string child_frame_id;
  std::int64_t stamp_ns = 0;
};

// Walk every TF topic once: insert each contained TransformStamped into
// `buffer` (static or dynamic per topic name) and, for messages on
// `input_topic`, record the (frame_id, child_frame_id, stamp_ns) so the
// caller can later filter by chain-edge membership without a second
// bag pass.
void load_tf_buffer_and_input_edges(
  const std::filesystem::path & bag_path, const std::vector<TfTopic> & tf_topics,
  const std::string & input_topic, tf2::BufferCore & buffer, std::vector<InputEdge> & input_edges)
{
  auto reader = io::open_read(bag_path);
  io::ReadFilter filter;
  for (const auto & t : tf_topics) {
    filter.topics.push_back(t.name);
  }
  reader->set_filter(filter);

  std::unordered_map<std::string, bool> is_static_by_topic;
  for (const auto & t : tf_topics) {
    is_static_by_topic[t.name] = t.is_static;
  }

  std::unordered_map<std::string, std::unique_ptr<core::decoder::Decoder>> decoder_by_topic;
  for (const auto & topic_info : reader->topics()) {
    if (topic_info.type != kTfMessageType) {
      continue;
    }
    if (is_static_by_topic.find(topic_info.name) == is_static_by_topic.end()) {
      continue;
    }
    auto open = core::decoder::open_decoder(topic_info);
    if (!open.ok()) {
      throw std::runtime_error(
        "Could not open decoder for TF topic '" + topic_info.name + "': " + open.error);
    }
    decoder_by_topic.emplace(topic_info.name, std::move(open.decoder));
  }

  // Refuse to merge TF that disagrees: a child given different parents by two
  // topics, or a child declared by both a static and a dynamic topic. The
  // checker is cross-topic only, so a single topic's own time series is fine.
  core::TfMergeConflictChecker conflict_checker;

  io::RawMessage raw;
  while (reader->next(raw)) {
    auto it = decoder_by_topic.find(raw.topic->name);
    if (it == decoder_by_topic.end()) {
      continue;
    }
    const auto decoded = it->second->decode(raw.payload);
    if (!decoded.ok()) {
      throw std::runtime_error(
        "Failed to decode TF message on '" + raw.topic->name + "': " + decoded.error);
    }
    const auto transforms = core::extract_tf_message(*decoded.value);
    const bool is_static = is_static_by_topic.at(raw.topic->name);
    const bool is_input = (raw.topic->name == input_topic);
    for (const auto & t : transforms) {
      if (
        const auto conflict =
          conflict_checker.add(t.header.frame_id, t.child_frame_id, raw.topic->name, is_static)) {
        throw std::runtime_error("TF merge conflict: " + *conflict);
      }
      buffer.setTransform(t, "bagwiz", is_static);
      if (is_input) {
        const std::int64_t ns = static_cast<std::int64_t>(t.header.stamp.sec) * 1'000'000'000LL +
                                static_cast<std::int64_t>(t.header.stamp.nanosec);
        input_edges.push_back({t.header.frame_id, t.child_frame_id, ns});
      }
    }
  }
}

// One decoded sample from a pose / odometry input topic. `pose` carries the
// tracked body expressed in its own header.frame_id. `child_frame` is the
// Odometry child_frame_id (the body's frame name); it is empty for the pose
// topics, which do not name their body.
struct PoseSample
{
  geometry_msgs::msg::PoseStamped pose;
  std::string child_frame;
};

// Decode one input message into a PoseSample according to `kind`. Returns
// false (and sets `skip_reason`) when the payload does not match the expected
// shape; the caller counts that as a skipped sample rather than aborting.
bool decode_pose_sample(
  PoseDumpKind kind, const core::cdr_walker::Value & value, PoseSample & out,
  std::string & skip_reason)
{
  switch (kind) {
    case PoseDumpKind::PoseStamped: {
      const auto ps = core::extract_pose_stamped_message(value);
      if (!ps.has_value()) {
        skip_reason = "could not parse PoseStamped";
        return false;
      }
      out.pose = *ps;
      return true;
    }
    case PoseDumpKind::PoseWithCovarianceStamped: {
      const auto pwc = core::extract_pose_with_covariance_stamped_message(value);
      if (!pwc.has_value()) {
        skip_reason = "could not parse PoseWithCovarianceStamped";
        return false;
      }
      out.pose.header = pwc->header;
      out.pose.pose = pwc->pose.pose;
      return true;
    }
    case PoseDumpKind::Odometry: {
      const auto odom = core::extract_odometry_message(value);
      if (!odom.has_value()) {
        skip_reason = "could not parse Odometry";
        return false;
      }
      out.pose.header = odom->header;
      out.pose.pose = odom->pose.pose;
      out.child_frame = odom->child_frame_id;
      return true;
    }
  }
  return false;
}

}  // namespace

// `bagwiz traj` is a command group for trajectory-shaped operations.
//
// Subcommands
// -----------
//   dump      Write TUM trajectory samples. Every row is the pose of the
//             tracked frame --to expressed in the reference frame --from,
//             resolved through the bag's TF tree (static + dynamic). For
//             tf2_msgs/msg/TFMessage, --from/--to are required and sampling
//             follows TF chain edges on the input topic. For nav_msgs/msg/
//             Odometry, both default per message (--from to header.frame_id,
//             --to to child_frame_id); a --to that differs from child_frame_id
//             traverses the TF tree (e.g. static base_link -> sensor). For
//             PoseStamped / PoseWithCovarianceStamped, --to is the asserted
//             body frame (the pose already encodes it, so it does not change
//             the numbers); --from optionally re-expresses each pose via TF.
//   join      Embed a trajectory file into a bag as a new TFMessage topic.
//             Each trajectory row becomes one message on <topic>, with the
//             message receive time and header.stamp taken from the row's
//             timestamp.
class TrajCommand : public Command
{
public:
  [[nodiscard]] std::string_view name() const override { return "traj"; }
  [[nodiscard]] std::string_view description() const override { return "Trajectory operations"; }

  void configure(CLI::App & app) override
  {
    app.require_subcommand(1);
    configure_dump(app);
    configure_join(app);
  }

  int run() override
  {
    switch (selected_) {
      case Subcommand::kDump:
        return run_dump();
      case Subcommand::kJoin:
        return run_join();
      case Subcommand::kNone:
        BAGWIZ_LOG_ERROR(kLogger, "no subcommand selected");
        return 1;
    }
    return 1;
  }

private:
  enum class Subcommand { kNone, kDump, kJoin };
  Subcommand selected_ = Subcommand::kNone;

  struct DumpArgs
  {
    std::filesystem::path input_path;
    std::string topic;
    std::filesystem::path output_path;
    std::string format;
    std::optional<std::string> from_frame;
    std::optional<std::string> to_frame;
    bool overwrite = false;  // replace any pre-existing output_path
  } dump_args_;

  struct JoinArgs
  {
    std::filesystem::path input_path;
    std::filesystem::path traj_path;
    std::string topic;
    std::optional<std::filesystem::path> output_path;
    std::string format;
    std::string msg_type = kJoinMsgTypeTf;
    std::optional<std::string> from_frame;
    std::optional<std::string> to_frame;
    bool force = false;
    bool overwrite = false;  // replace any pre-existing -o/--output path
                             // (no effect in in-place mode, where <input> is
                             // already the target by definition)
  } join_args_;

  void configure_dump(CLI::App & app)
  {
    auto * sub = app.add_subcommand(
      "dump",
      "Dump a topic's trajectory to a file. The accepted message types and the meaning "
      "of --from / --to per type are listed in the SUPPORTED TOPIC TYPES section below.");
    sub->add_option("input", dump_args_.input_path, "Bag path (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    sub
      ->add_option(
        "topic", dump_args_.topic,
        "Topic to sample (e.g. /tf, /localization/pose). See SUPPORTED TOPIC TYPES below.")
      ->required();
    sub->add_option("output", dump_args_.output_path, "Output file path")->required();
    sub
      ->add_option(
        "-f,--format", dump_args_.format,
        "Output format (tum). When omitted, inferred from the output file extension (e.g. .tum).")
      ->check(CLI::IsMember({kFormatTum}));
    sub->add_option(
      "--from", dump_args_.from_frame,
      "Reference frame the trajectory is expressed in. Required for TF topics; "
      "optional for pose / odometry (defaults to each message's header.frame_id). "
      "See SUPPORTED TOPIC TYPES below.");
    sub->add_option(
      "--to", dump_args_.to_frame,
      "Tracked frame whose trajectory is written. Required for TF topics; optional "
      "for odometry (defaults to child_frame_id; a different value traverses the TF "
      "tree). For pose topics it names the body the pose reports. See SUPPORTED "
      "TOPIC TYPES below.");
    sub->add_flag(
      "-w,--overwrite", dump_args_.overwrite,
      "Replace <output> if it already exists. Without this flag, an "
      "existing output path stops the run.");
    sub->footer(
      "SUPPORTED TOPIC TYPES:\n"
      "  Every row is the pose of --to expressed in --from. All TF lookups are\n"
      "  resolved automatically from the bag's static and dynamic TFs (/tf and\n"
      "  *tf_static are picked up from the bag) — any multi-hop path through the TF\n"
      "  tree is OK; --from / --to need not be directly connected.\n"
      "\n"
      "  tf2_msgs/msg/TFMessage  (e.g. /tf)\n"
      "    --from  REQUIRED  reference frame of the trajectory\n"
      "    --to    REQUIRED  tracked frame\n"
      "\n"
      "  nav_msgs/msg/Odometry\n"
      "    --from  optional  reference frame; defaults to header.frame_id (no remap)\n"
      "    --to    optional  tracked frame; defaults to child_frame_id. A value that\n"
      "                      differs from child_frame_id walks the TF tree from the\n"
      "                      body to --to (e.g. static base_link -> sensor)\n"
      "\n"
      "  geometry_msgs/msg/PoseStamped\n"
      "  geometry_msgs/msg/PoseWithCovarianceStamped\n"
      "    --from  optional  re-express each pose into this frame via TF;\n"
      "                      when omitted, the pose's header.frame_id is kept as-is\n"
      "    --to    optional  the body frame the pose reports. The pose already\n"
      "                      encodes its body, so --to does not change the numbers\n"
      "                      and never traverses further; use Odometry or /tf for\n"
      "                      tracked-side TF traversal");
    sub->callback([this]() { selected_ = Subcommand::kDump; });
  }

  int run_dump_tf_message(const DumpArgs & args)
  {
    if (
      !args.from_frame.has_value() || !args.to_frame.has_value() || args.from_frame->empty() ||
      args.to_frame->empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Topic '%s' is tf2_msgs/msg/TFMessage; both --from and --to are required.",
        args.topic.c_str());
      return 1;
    }
    if (*args.from_frame == *args.to_frame) {
      BAGWIZ_LOG_ERROR(
        kLogger, "--from and --to must be distinct frames; both were '%s'.",
        args.from_frame->c_str());
      return 1;
    }

    std::unique_ptr<io::BagReader> reader;
    try {
      reader = io::open_read(args.input_path);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to open %s: %s", args.input_path.c_str(), e.what());
      return 1;
    }

    // /tf_static is one-shot and not a sensible sampling source. The
    // assumption is "1 dynamic + 1 static topic per bag", so the user
    // is expected to point at the dynamic side.
    if (is_static_tf_topic(args.topic)) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "Topic '%s' looks like a static TF topic (name ends with 'tf_static'). "
        "Pass the dynamic /tf-style topic instead; static TF in the bag is loaded automatically.",
        args.topic.c_str());
      return 1;
    }

    const auto tf_topics = collect_tf_topics(*reader);
    if (tf_topics.empty()) {
      BAGWIZ_LOG_ERROR(kLogger, "Bag has no tf2_msgs/msg/TFMessage topics; nothing to dump.");
      return 1;
    }

    tf2::BufferCore tf_buffer{std::chrono::hours(24 * 365)};
    std::vector<InputEdge> input_edges;
    try {
      load_tf_buffer_and_input_edges(
        args.input_path, tf_topics, args.topic, tf_buffer, input_edges);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to load TF from the bag: %s", e.what());
      return 1;
    }

    if (input_edges.empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Topic '%s' carried no TransformStamped entries; nothing to dump.",
        args.topic.c_str());
      return 1;
    }

    const tf2::TimePoint resolve_tp{std::chrono::nanoseconds(input_edges.front().stamp_ns)};
    const auto chain = core::resolve_chain(tf_buffer, *args.from_frame, *args.to_frame, resolve_tp);
    if (chain.empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "No TF path between '%s' and '%s' in the bag (different connected components, "
        "or one of the frames is absent).",
        args.from_frame->c_str(), args.to_frame->c_str());
      return 1;
    }
    const auto path_edges = core::chain_to_edges(tf_buffer, chain, resolve_tp);

    auto edge_key = [](const std::string & a, const std::string & b) { return a + '\0' + b; };
    std::unordered_set<std::string> path_edge_set;
    path_edge_set.reserve(path_edges.size());
    for (const auto & e : path_edges) {
      path_edge_set.insert(edge_key(e.first, e.second));
    }

    std::vector<std::int64_t> sample_stamps;
    sample_stamps.reserve(input_edges.size());
    for (const auto & ie : input_edges) {
      if (path_edge_set.count(edge_key(ie.frame_id, ie.child_frame_id)) != 0) {
        sample_stamps.push_back(ie.stamp_ns);
      }
    }

    if (sample_stamps.empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "No chain edges between '%s' and '%s' are published on '%s'. "
        "The whole chain may live on /tf_static (a fully-static path is rejected by "
        "traj dump — there is no time axis to sample), or the input topic does not "
        "carry the relevant edge.",
        args.from_frame->c_str(), args.to_frame->c_str(), args.topic.c_str());
      return 1;
    }

    std::sort(sample_stamps.begin(), sample_stamps.end());
    sample_stamps.erase(
      std::unique(sample_stamps.begin(), sample_stamps.end()), sample_stamps.end());

    std::vector<core::TrajectoryPose> poses;
    poses.reserve(sample_stamps.size());
    std::int64_t skipped = 0;
    std::string last_skip_reason;
    for (const std::int64_t ns : sample_stamps) {
      const tf2::TimePoint tp{std::chrono::nanoseconds(ns)};
      try {
        const auto tf = tf_buffer.lookupTransform(*args.from_frame, *args.to_frame, tp);
        core::TrajectoryPose p;
        p.timestamp_ns = ns;
        p.tx = tf.transform.translation.x;
        p.ty = tf.transform.translation.y;
        p.tz = tf.transform.translation.z;
        p.qx = tf.transform.rotation.x;
        p.qy = tf.transform.rotation.y;
        p.qz = tf.transform.rotation.z;
        p.qw = tf.transform.rotation.w;
        poses.push_back(p);
      } catch (const tf2::TransformException & e) {
        ++skipped;
        last_skip_reason = e.what();
      }
    }

    if (poses.empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger, "All %zu sample stamps failed to resolve via lookupTransform. Last reason: %s",
        sample_stamps.size(),
        last_skip_reason.empty() ? "(none recorded)" : last_skip_reason.c_str());
      return 1;
    }

    std::ofstream out(args.output_path, std::ios::out | std::ios::trunc);
    if (!out) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Failed to open output path %s for writing", args.output_path.c_str());
      return 1;
    }
    core::write_tum(out, poses);
    out.close();

    BAGWIZ_LOG_INFO(
      kLogger, "Wrote %zu poses (from %zu sample stamps, %" PRId64 " skipped) to %s in %s format",
      poses.size(), sample_stamps.size(), skipped, args.output_path.c_str(), args.format.c_str());
    return 0;
  }

  // Build the full TF tree (every TFMessage topic in the bag: *tf_static as
  // static, the rest as dynamic) into `tf_buffer`. Returns false (after
  // logging) when a lookup is requested but the bag carries no TF to resolve
  // it. A no-op (returns true) when `need_tree` is false.
  bool build_dump_tf_tree(const DumpArgs & args, bool need_tree, tf2::BufferCore & tf_buffer)
  {
    if (!need_tree) {
      return true;
    }
    std::unique_ptr<io::BagReader> reader;
    try {
      reader = io::open_read(args.input_path);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to open %s: %s", args.input_path.c_str(), e.what());
      return false;
    }
    const auto tf_topics = collect_tf_topics(*reader);
    if (tf_topics.empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "Topic '%s' needs a TF lookup (--from / --to) but the bag has no "
        "tf2_msgs/msg/TFMessage topics to resolve it.",
        args.topic.c_str());
      return false;
    }
    try {
      // input_topic = "" : load every TF topic into the buffer, record no edges.
      std::vector<InputEdge> ignored_edges;
      load_tf_buffer_and_input_edges(
        args.input_path, tf_topics, std::string{}, tf_buffer, ignored_edges);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to load TF from the bag: %s", e.what());
      return false;
    }
    return true;
  }

  // Unified pose / odometry trajectory dump.
  //
  // Each output row is the pose of the tracked frame `--to` expressed in the
  // reference frame `--from`, composed as
  //
  //   T_from_to = T_from_header * T_header_body * T_body_to
  //
  // where T_header_body is the message's own pose and the two bridges come
  // from the bag's TF tree (static + dynamic). Frames default per message:
  // `--from` to header.frame_id (no remap) and, for Odometry, `--to` to
  // child_frame_id (no tracked-side traversal). PoseStamped / PWC do not name
  // their body, so `--to` is accepted as the asserted body frame but never
  // traverses further (the pose already encodes the body); only `--from`
  // re-expresses them.
  int run_dump_pose_topic(
    const DumpArgs & args, const io::TopicInfo & topic_info, PoseDumpKind kind)
  {
    const bool is_odom = (kind == PoseDumpKind::Odometry);

    if (args.from_frame.has_value() && args.from_frame->empty()) {
      BAGWIZ_LOG_ERROR(kLogger, "When set, --from must be a non-empty frame id.");
      return 1;
    }
    if (args.to_frame.has_value() && args.to_frame->empty()) {
      BAGWIZ_LOG_ERROR(kLogger, "When set, --to must be a non-empty frame id.");
      return 1;
    }

    // A TF lookup can be non-identity only when --from is set (reference-side
    // bridge) or Odometry has --to set (tracked-side bridge). Pure raw dumps
    // (no flags) need no TF tree at all.
    const bool need_tree = args.from_frame.has_value() || (is_odom && args.to_frame.has_value());

    tf2::BufferCore tf_buffer{std::chrono::hours(24 * 365)};
    if (!build_dump_tf_tree(args, need_tree, tf_buffer)) {
      return 1;
    }

    std::unique_ptr<io::BagReader> reader;
    try {
      reader = io::open_read(args.input_path);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to open %s: %s", args.input_path.c_str(), e.what());
      return 1;
    }
    io::ReadFilter filter;
    filter.topics.push_back(args.topic);
    reader->set_filter(filter);

    std::unique_ptr<core::decoder::Decoder> decoder;
    for (const auto & ti : reader->topics()) {
      if (ti.name != args.topic) {
        continue;
      }
      auto open = core::decoder::open_decoder(ti);
      if (!open.ok()) {
        BAGWIZ_LOG_ERROR(
          kLogger, "Could not open decoder for topic '%s': %s", ti.name.c_str(),
          open.error.c_str());
        return 1;
      }
      decoder = std::move(open.decoder);
      break;
    }
    if (!decoder) {
      BAGWIZ_LOG_ERROR(kLogger, "Could not open decoder for topic '%s'.", args.topic.c_str());
      return 1;
    }

    std::vector<core::TrajectoryPose> poses;
    std::int64_t skipped = 0;
    std::string last_skip_reason;

    io::RawMessage raw;
    while (reader->next(raw)) {
      if (raw.topic->name != args.topic) {
        continue;
      }
      const auto decoded = decoder->decode(raw.payload);
      if (!decoded.ok()) {
        BAGWIZ_LOG_ERROR(
          kLogger, "Failed to decode message on '%s': %s", raw.topic->name.c_str(),
          decoded.error.c_str());
        return 1;
      }

      PoseSample sample;
      if (!decode_pose_sample(kind, *decoded.value, sample, last_skip_reason)) {
        ++skipped;
        continue;
      }
      if (sample.pose.header.frame_id.empty()) {
        BAGWIZ_LOG_ERROR(
          kLogger, "Topic '%s': message has empty header.frame_id (required for %s).",
          args.topic.c_str(), topic_info.type.c_str());
        return 1;
      }
      if (is_odom && sample.child_frame.empty()) {
        BAGWIZ_LOG_ERROR(
          kLogger, "Topic '%s': nav_msgs/msg/Odometry requires a non-empty child_frame_id.",
          args.topic.c_str());
        return 1;
      }

      const std::string & header_frame = sample.pose.header.frame_id;
      const std::string from_frame = args.from_frame.has_value() ? *args.from_frame : header_frame;
      const std::int64_t ns =
        static_cast<std::int64_t>(sample.pose.header.stamp.sec) * 1'000'000'000LL +
        static_cast<std::int64_t>(sample.pose.header.stamp.nanosec);
      const tf2::TimePoint tp{std::chrono::nanoseconds(ns)};

      try {
        // Reference-side bridge: re-express the result into --from. Identity
        // (no lookup) when --from is absent or already equals header.frame_id.
        std::optional<geometry_msgs::msg::Transform> from_header;
        if (from_frame != header_frame) {
          from_header = tf_buffer.lookupTransform(from_frame, header_frame, tp).transform;
        }
        // Tracked-side bridge (Odometry only): walk body/child -> --to via the
        // TF tree (e.g. base_link -> sensor through static TF). Identity when
        // --to is absent or already equals the body frame.
        std::optional<geometry_msgs::msg::Transform> body_to;
        if (is_odom && args.to_frame.has_value() && *args.to_frame != sample.child_frame) {
          body_to = tf_buffer.lookupTransform(sample.child_frame, *args.to_frame, tp).transform;
        }

        const auto out_pose = core::compose_trajectory_pose(from_header, sample.pose.pose, body_to);
        core::TrajectoryPose p;
        p.timestamp_ns = ns;
        p.tx = out_pose.position.x;
        p.ty = out_pose.position.y;
        p.tz = out_pose.position.z;
        p.qx = out_pose.orientation.x;
        p.qy = out_pose.orientation.y;
        p.qz = out_pose.orientation.z;
        p.qw = out_pose.orientation.w;
        poses.push_back(p);
      } catch (const tf2::TransformException & e) {
        ++skipped;
        last_skip_reason = e.what();
      }
    }

    if (poses.empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "No poses written for topic '%s'. Skipped/failed samples: %" PRId64 ". Last note: %s",
        args.topic.c_str(), skipped,
        last_skip_reason.empty() ? "(none)" : last_skip_reason.c_str());
      return 1;
    }

    std::ofstream out(args.output_path, std::ios::out | std::ios::trunc);
    if (!out) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Failed to open output path %s for writing", args.output_path.c_str());
      return 1;
    }
    core::write_tum(out, poses);
    out.close();

    BAGWIZ_LOG_INFO(
      kLogger, "Wrote %zu poses (%" PRId64 " skipped) to %s in %s format", poses.size(), skipped,
      args.output_path.c_str(), args.format.c_str());
    return 0;
  }

  int run_dump()
  {
    std::string resolved_format;
    if (!resolve_dump_format(dump_args_.format, dump_args_.output_path, resolved_format)) {
      return 1;
    }
    dump_args_.format = std::move(resolved_format);

    if (const auto r = core::prepare_output_path(dump_args_.output_path, dump_args_.overwrite);
        !r.ok) {
      BAGWIZ_LOG_ERROR(kLogger, "%s", r.error.c_str());
      return 1;
    }

    const auto & args = dump_args_;

    std::unique_ptr<io::BagReader> reader;
    try {
      reader = io::open_read(args.input_path);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to open %s: %s", args.input_path.c_str(), e.what());
      return 1;
    }

    const io::TopicInfo * topic_info = nullptr;
    for (const auto & t : reader->topics()) {
      if (t.name == args.topic) {
        topic_info = &t;
        break;
      }
    }
    if (topic_info == nullptr) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Topic '%s' is not present in %s", args.topic.c_str(), args.input_path.c_str());
      return 1;
    }

    if (topic_info->type == kTfMessageType) {
      return run_dump_tf_message(args);
    }
    if (topic_info->type == kPoseStampedType) {
      return run_dump_pose_topic(args, *topic_info, PoseDumpKind::PoseStamped);
    }
    if (topic_info->type == kPoseWithCovarianceStampedType) {
      return run_dump_pose_topic(args, *topic_info, PoseDumpKind::PoseWithCovarianceStamped);
    }
    if (topic_info->type == kOdometryType) {
      return run_dump_pose_topic(args, *topic_info, PoseDumpKind::Odometry);
    }

    BAGWIZ_LOG_ERROR(
      kLogger,
      "Topic '%s' has unsupported type '%s'. Supported: tf2_msgs/msg/TFMessage, "
      "geometry_msgs/msg/PoseStamped, geometry_msgs/msg/PoseWithCovarianceStamped, "
      "nav_msgs/msg/Odometry.",
      args.topic.c_str(), topic_info->type.c_str());
    return 1;
  }

  // Resolve the trajectory file's format. --format wins; otherwise pull
  // it from the input file's extension. The function is structured so
  // adding new formats does not require touching the caller.
  static bool resolve_join_format(
    const std::string & format_opt, const std::filesystem::path & traj_path, std::string & out)
  {
    if (!format_opt.empty()) {
      if (format_opt != kFormatTum) {
        BAGWIZ_LOG_ERROR(
          kLogger, "Unsupported trajectory format '%s'. Supported: %s.", format_opt.c_str(),
          kFormatTum);
        return false;
      }
      out = format_opt;
      return true;
    }

    const std::string ext = output_path_extension_lower(traj_path);
    if (ext.empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "Trajectory format is not set (-f/--format) and the trajectory path '%s' has no usable "
        "extension; use e.g. '*.tum' or pass --format %s.",
        traj_path.c_str(), kFormatTum);
      return false;
    }
    if (ext == kFormatTum) {
      out = kFormatTum;
      return true;
    }

    BAGWIZ_LOG_ERROR(
      kLogger,
      "Trajectory format is not set (-f/--format) and extension '.%s' is not recognized; "
      "use '*.tum' or pass --format %s.",
      ext.c_str(), kFormatTum);
    return false;
  }

  void configure_join(CLI::App & app)
  {
    auto * sub = app.add_subcommand(
      "join",
      "Embed a trajectory file into a bag as a new topic. Each row in the trajectory becomes "
      "one ROS message published on <topic>; both the message's receive time and the in-message "
      "header.stamp are taken from the trajectory's per-row timestamp.");
    sub->add_option("input", join_args_.input_path, "Bag path (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    sub
      ->add_option(
        "traj_file", join_args_.traj_path,
        "Trajectory file. Format is selected by -f/--format, or inferred from the file extension "
        "when -f is omitted.")
      ->required()
      ->check(CLI::ExistingFile);
    sub
      ->add_option(
        "topic", join_args_.topic,
        "Topic name to embed the trajectory under. When the topic already exists in <input>, "
        "pass --force to drop its existing messages and replace them.")
      ->required();
    sub->add_option(
      "-o,--output", join_args_.output_path,
      "Output bag path. When omitted, <input> is replaced in place via a sibling tmp directory.");
    sub
      ->add_option(
        "-f,--format", join_args_.format,
        "Trajectory format id. When omitted, inferred from the trajectory file's extension.")
      ->check(CLI::IsMember({kFormatTum}));
    sub
      ->add_option(
        "-t,--msg-type", join_args_.msg_type,
        "ROS message type to publish under <topic>. Only 'tf' (tf2_msgs/msg/TFMessage) is "
        "supported today.")
      ->check(CLI::IsMember({kJoinMsgTypeTf}));
    sub->add_option(
      "--from", join_args_.from_frame,
      "Parent frame id. Required for --msg-type tf; mapped to TransformStamped.header.frame_id.");
    sub->add_option(
      "--to", join_args_.to_frame,
      "Child frame id. Required for --msg-type tf; mapped to TransformStamped.child_frame_id.");
    sub
      ->add_flag(
        "--force", join_args_.force,
        "Overwrite when <topic> already carries messages in <input>; otherwise the command "
        "aborts.")
      ->default_val(false);
    sub->add_flag(
      "-w,--overwrite", join_args_.overwrite,
      "Replace -o/--output if it already exists. Has no effect in in-place "
      "mode (when -o is omitted, <input> is replaced atomically by design).");
    sub->callback([this]() { selected_ = Subcommand::kJoin; });
  }

  // Read the trajectory file into a vector of TransformStamped using
  // the resolved format. Dispatch by `resolved_format`; new formats
  // hook in here without changing the caller.
  static bool load_trajectory_as_transforms(
    const JoinArgs & args, const std::string & resolved_format,
    std::vector<geometry_msgs::msg::TransformStamped> & out_transforms,
    std::vector<std::int64_t> & out_stamps_ns)
  {
    if (resolved_format != kFormatTum) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Internal: resolved trajectory format '%s' is not handled.",
        resolved_format.c_str());
      return false;
    }

    std::ifstream in(args.traj_path);
    if (!in) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Failed to open trajectory file '%s' for reading.", args.traj_path.c_str());
      return false;
    }
    const auto parsed = core::read_tum(in);
    if (parsed.poses.empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "Trajectory file '%s' contains no valid TUM rows (skipped %" PRId64 " malformed line(s)).",
        args.traj_path.c_str(), parsed.skipped_lines);
      return false;
    }
    if (parsed.skipped_lines > 0) {
      BAGWIZ_LOG_WARN(
        kLogger, "Trajectory file '%s': skipped %" PRId64 " malformed line(s).",
        args.traj_path.c_str(), parsed.skipped_lines);
    }

    out_transforms.reserve(parsed.poses.size());
    out_stamps_ns.reserve(parsed.poses.size());
    for (const auto & p : parsed.poses) {
      out_transforms.push_back(
        core::pose_to_transform_stamped(p, *args.from_frame, *args.to_frame));
      out_stamps_ns.push_back(p.timestamp_ns);
    }
    return true;
  }

  // Execute one full pass: open reader, plan the topic conflict,
  // declare topics on the writer, stream-copy with suppression, append
  // the injected payloads. Used for both in-place and explicit-output
  // modes; the writer factory is parameterised so write_bag_inplace can
  // hand in a tmp path.
  static int execute_join_pass(
    const JoinArgs & args, std::span<const geometry_msgs::msg::TransformStamped> transforms,
    std::span<const std::int64_t> stamps_ns,
    const std::function<std::unique_ptr<io::BagWriter>()> & open_writer)
  {
    constexpr const char * kExpectedType = "tf2_msgs/msg/TFMessage";

    std::unique_ptr<io::BagReader> reader;
    try {
      reader = io::open_read(args.input_path);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to open %s: %s", args.input_path.c_str(), e.what());
      return 1;
    }

    // Snapshot the input's topic list for conflict detection; the reader's
    // span is invalidated by subsequent operations.
    const std::vector<io::TopicInfo> input_topics_pre(
      reader->topics().begin(), reader->topics().end());

    std::int64_t existing_count = 0;
    try {
      const std::vector<std::string> count_topics{args.topic};
      const auto topic_counts = reader->compute_topic_counts(count_topics);
      if (auto it = topic_counts.find(args.topic); it != topic_counts.end()) {
        existing_count = it->second;
      }
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Failed to compute topic count on %s: %s", args.input_path.c_str(), e.what());
      return 1;
    }

    const auto decision = core::decide_topic_write(
      input_topics_pre, args.topic, kExpectedType, existing_count, args.force);
    switch (decision.action) {
      case core::TopicWriteAction::kConflictAbort:
      case core::TopicWriteAction::kTypeMismatch:
        BAGWIZ_LOG_ERROR(kLogger, "%s", decision.reason.c_str());
        return 1;
      case core::TopicWriteAction::kDeclareAndSuppress:
        BAGWIZ_LOG_WARN(kLogger, "%s", decision.reason.c_str());
        break;
      case core::TopicWriteAction::kDeclareNew:
      case core::TopicWriteAction::kDeclareKeep:
        // Quiet — these are the expected paths.
        break;
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

    // Snapshot the input's topic list after schema backfill so the output
    // writer receives embedded schemas.
    const std::vector<io::TopicInfo> input_topics(reader->topics().begin(), reader->topics().end());

    // Declare every existing topic from the input. When the action is
    // kDeclareNew, also declare a freshly-synthesised TopicInfo for
    // <topic>. When kDeclareKeep / kDeclareAndSuppress, the matching
    // input topic is already in the declare loop.
    for (const auto & t : input_topics) {
      try {
        writer->declare_topic(t);
      } catch (const std::exception & e) {
        BAGWIZ_LOG_ERROR(kLogger, "declare_topic failed for '%s': %s", t.name.c_str(), e.what());
        return 1;
      }
    }
    if (decision.action == core::TopicWriteAction::kDeclareNew) {
      try {
        writer->declare_topic(core::make_tf_message_topic_info(args.topic));
      } catch (const std::exception & e) {
        BAGWIZ_LOG_ERROR(
          kLogger, "declare_topic failed for new topic '%s': %s", args.topic.c_str(), e.what());
        return 1;
      }
    }

    std::unordered_set<std::string> suppress;
    if (decision.action == core::TopicWriteAction::kDeclareAndSuppress) {
      suppress.insert(args.topic);
    }

    core::BagCopyCounts counts;
    try {
      counts = core::bag_copy_filtered(
        *reader, *writer, suppress, "traj join", core::pipeline::BackendKind::Pipelined);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Stream copy from %s failed: %s", args.input_path.c_str(), e.what());
      return 1;
    }

    std::uint64_t injected = 0;
    for (std::size_t i = 0; i < transforms.size(); ++i) {
      std::vector<std::byte> payload;
      try {
        payload = core::serialize_tf_message(
          std::span<const geometry_msgs::msg::TransformStamped>(transforms.data() + i, 1));
      } catch (const std::exception & e) {
        BAGWIZ_LOG_ERROR(kLogger, "Failed to serialize TFMessage for sample #%zu: %s", i, e.what());
        return 1;
      }
      try {
        writer->write(
          args.topic, stamps_ns[i], std::span<const std::byte>(payload.data(), payload.size()));
      } catch (const std::exception & e) {
        BAGWIZ_LOG_ERROR(
          kLogger, "Failed to write TFMessage on '%s' at stamp %" PRId64 ": %s", args.topic.c_str(),
          stamps_ns[i], e.what());
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
      "traj join: copied %" PRIu64 " message(s), suppressed %" PRIu64 ", injected %" PRIu64
      " TFMessage(s) on '%s'.",
      counts.copied, counts.suppressed, injected, args.topic.c_str());
    return 0;
  }

  int run_join()
  {
    const auto & args = join_args_;

    // 1. Validate type-specific arg constraints. Today only --msg-type tf
    //    is supported, and it requires --from / --to as distinct non-empty
    //    frame ids.
    if (args.msg_type == kJoinMsgTypeTf) {
      if (
        !args.from_frame.has_value() || !args.to_frame.has_value() || args.from_frame->empty() ||
        args.to_frame->empty()) {
        BAGWIZ_LOG_ERROR(
          kLogger,
          "--msg-type tf requires both --from (parent frame_id) and --to (child_frame_id).");
        return 1;
      }
      if (*args.from_frame == *args.to_frame) {
        BAGWIZ_LOG_ERROR(
          kLogger, "--from and --to must be distinct frames; both were '%s'.",
          args.from_frame->c_str());
        return 1;
      }
    }

    // 2. Resolve the trajectory format.
    std::string resolved_format;
    if (!resolve_join_format(args.format, args.traj_path, resolved_format)) {
      return 1;
    }

    // 3. Read the trajectory into TransformStamped[] + stamp[].
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    std::vector<std::int64_t> stamps_ns;
    if (!load_trajectory_as_transforms(args, resolved_format, transforms, stamps_ns)) {
      return 1;
    }

    // 4. Pick the writer factory based on -o presence.
    if (args.output_path.has_value()) {
      // Explicit -o: let the factory pick from the user-provided path's
      // extension (e.g. .mcap / .db3 / directory).
      auto make_writer = [](const std::filesystem::path & out_path) {
        io::CreateOptions copts;
        copts.format = io::Format::Auto;
        copts.layout = io::Layout::Auto;
        copts.mcap_compression = "none";
        return io::open_write(out_path, copts);
      };
      if (const auto r = core::prepare_output_path(*args.output_path, args.overwrite); !r.ok) {
        BAGWIZ_LOG_ERROR(kLogger, "%s", r.error.c_str());
        return 1;
      }
      return execute_join_pass(
        args, transforms, stamps_ns, [&]() { return make_writer(*args.output_path); });
    }

    // In-place mode: pin format and layout to the input's identity. The
    // tmp path used by write_bag_inplace carries a synthetic suffix
    // (".bagwiz-inplace-tmp-..."), so Format::Auto / Layout::Auto would
    // misread it as a directory MCAP target and silently convert db3
    // inputs to mcap on swap.
    const auto inplace_copts = io::create_options_preserving_storage(args.input_path);
    if (inplace_copts.format == io::Format::Auto) {
      BAGWIZ_LOG_ERROR(
        kLogger, "traj join: could not detect storage format of input bag '%s'.",
        args.input_path.string().c_str());
      return 1;
    }
    auto make_inplace_writer = [inplace_copts](const std::filesystem::path & out_path) {
      auto copts = inplace_copts;
      copts.mcap_compression = "none";
      return io::open_write(out_path, copts);
    };

    // Hand off to write_bag_inplace, which produces the tmp path. The
    // closure runs execute_join_pass against the tmp; because
    // execute_join_pass returns int rather than throwing on command-level
    // errors, we surface non-zero exits via a captured status and
    // translate them into a runtime_error so the in-place helper aborts
    // the swap.
    int pass_status = 0;
    try {
      core::write_bag_inplace(args.input_path, [&](const std::filesystem::path & tmp) {
        pass_status = execute_join_pass(
          args, transforms, stamps_ns, [&]() { return make_inplace_writer(tmp); });
        if (pass_status != 0) {
          throw std::runtime_error("traj join: pass failed; aborting in-place swap");
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
};

BAGWIZ_REGISTER_COMMAND(TrajCommand)

}  // namespace bagwiz::commands
