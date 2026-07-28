// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/core/bag/bag_copy.hpp"
#include "bagwiz/core/bag/bag_topic_plan.hpp"
#include "bagwiz/core/bag/rewrite.hpp"
#include "bagwiz/core/bag/write_order.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/base/output_path.hpp"
#include "bagwiz/core/cdr_walker/value.hpp"
#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/tf/tf_buffer_loader.hpp"
#include "bagwiz/core/tf/tf_chain.hpp"
#include "bagwiz/core/tf/tf_merge_check.hpp"
#include "bagwiz/core/tf/tf_message_wire.hpp"
#include "bagwiz/core/tf/tf_topics.hpp"
#include "bagwiz/core/tf/tf_trajectory_sample.hpp"
#include "bagwiz/core/tf/tf_value_extract.hpp"
#include "bagwiz/core/tf/trajectory.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/bag_open.hpp"
#include "traj_common.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2/buffer_core.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <fmt/core.h>

#include <cctype>
#include <chrono>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
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

// --- traj join pass phases -------------------------------------------------

// Count the messages the input already carries on `topic` (0 when the topic
// is absent). Logs and returns std::nullopt when the count itself fails.
std::optional<std::int64_t> count_join_topic_messages(
  io::BagReader & reader, const std::string & topic, const std::filesystem::path & input_path)
{
  try {
    const std::vector<std::string> count_topics{topic};
    const auto topic_counts = reader.compute_topic_counts(count_topics);
    if (auto it = topic_counts.find(topic); it != topic_counts.end()) {
      return it->second;
    }
    return 0;
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(
      kLogger, "Failed to compute topic count on %s: %s", input_path.c_str(), e.what());
    return std::nullopt;
  }
}

// Surface the decide_topic_write outcome: conflict aborts and type mismatches
// are errors (the pass must stop), --force suppressions are warnings, and the
// expected paths stay quiet. Returns false when the pass must abort.
bool join_topic_decision_proceeds(const core::TopicWriteDecision & decision)
{
  switch (decision.action) {
    case core::TopicWriteAction::kConflictAbort:
    case core::TopicWriteAction::kTypeMismatch:
      BAGWIZ_LOG_ERROR(kLogger, "%s", decision.reason.c_str());
      return false;
    case core::TopicWriteAction::kDeclareAndSuppress:
      BAGWIZ_LOG_WARN(kLogger, "%s", decision.reason.c_str());
      return true;
    case core::TopicWriteAction::kDeclareNew:
    case core::TopicWriteAction::kDeclareKeep:
      // Quiet — these are the expected paths.
      return true;
  }
  return true;
}

// Declare every existing input topic on the writer (so embedded schemas
// round-trip). When the action is kDeclareNew, also declare a
// freshly-synthesised TopicInfo for `topic`; kDeclareKeep /
// kDeclareAndSuppress are covered by the input loop already.
bool declare_join_pass_topics(
  io::BagWriter & writer, std::span<const io::TopicInfo> input_topics,
  core::TopicWriteAction action, const std::string & topic)
{
  for (const auto & t : input_topics) {
    try {
      writer.declare_topic(t);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "declare_topic failed for '%s': %s", t.name.c_str(), e.what());
      return false;
    }
  }
  if (action == core::TopicWriteAction::kDeclareNew) {
    try {
      writer.declare_topic(core::make_tf_message_topic_info(topic));
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(
        kLogger, "declare_topic failed for new topic '%s': %s", topic.c_str(), e.what());
      return false;
    }
  }
  return true;
}

// Stream-copy the input to the writer, suppressing `topic`'s existing
// payloads when the decision said so. Logs and returns std::nullopt on
// failure.
std::optional<core::BagCopyCounts> copy_join_pass_messages(
  io::BagReader & reader, io::BagWriter & writer, core::TopicWriteAction action,
  const std::string & topic, const std::filesystem::path & input_path)
{
  std::unordered_set<std::string> suppress;
  if (action == core::TopicWriteAction::kDeclareAndSuppress) {
    suppress.insert(topic);
  }
  try {
    return core::bag_copy_filtered(
      reader, writer, suppress, "traj join", core::pipeline::BackendKind::Pipelined);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Stream copy from %s failed: %s", input_path.c_str(), e.what());
    return std::nullopt;
  }
}

// Serialize every trajectory sample as one TFMessage on `topic`, stamped at the
// sample's own time. Returns the messages ready to be merged into the copy;
// logs and returns std::nullopt on serialization failure.
//
// These are built before the copy rather than appended after it. The samples
// span the whole bag, so appending them would leave every trajectory row at the
// end of the storage order while its timestamp points back into the middle of
// the bag — delivered last to a consumer that reads in physical order instead
// of sorting. core::InjectingWriter merges them in time order instead.
std::optional<std::vector<core::OrderedMessage>> build_join_tf_messages(
  const std::string & topic, std::span<const geometry_msgs::msg::TransformStamped> transforms,
  std::span<const std::int64_t> stamps_ns)
{
  core::TfMessageSerializer serializer;
  std::vector<std::byte> payload;
  payload.reserve(256);
  std::vector<core::OrderedMessage> messages;
  messages.reserve(transforms.size());
  for (std::size_t i = 0; i < transforms.size(); ++i) {
    try {
      serializer.serialize_one(transforms[i], payload);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to serialize TFMessage for sample #%zu: %s", i, e.what());
      return std::nullopt;
    }
    messages.push_back(core::OrderedMessage{topic, stamps_ns[i], payload});
  }
  return messages;
}

}  // namespace

// `bagwiz traj` is a command group for trajectory-shaped operations.
//
// Subcommands
// -----------
//   dump      Write TUM trajectory samples. Every row is the pose of the
//             tracked frame --of expressed in the reference frame --ref,
//             resolved through the bag's TF tree (static + dynamic). For
//             tf2_msgs/msg/TFMessage, --ref/--of are required and sampling
//             follows TF chain edges on the input topic. For nav_msgs/msg/
//             Odometry, both default per message (--ref to header.frame_id,
//             --of to child_frame_id); a --of that differs from child_frame_id
//             traverses the TF tree (e.g. static base_link -> sensor). For
//             PoseStamped / PoseWithCovarianceStamped, --of is the asserted
//             body frame (the pose already encodes it, so it does not change
//             the numbers); --ref optionally re-expresses each pose via TF.
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
    std::optional<std::string> ref_frame;
    std::optional<std::string> of_frame;
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
    std::optional<std::string> ref_frame;
    std::optional<std::string> of_frame;
    bool force = false;
    bool overwrite = false;  // replace any pre-existing -o/--output path
                             // (no effect in in-place mode, where <input> is
                             // already the target by definition)
  } join_args_;

  void configure_dump(CLI::App & app)
  {
    auto * sub = app.add_subcommand(
      "dump",
      "Dump a topic's trajectory to a file. Every row is the pose of --of expressed in "
      "--ref, resolved through the bag's TF tree (static + dynamic).");
    sub->add_option("-i,--input", dump_args_.input_path, "Bag path (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    sub
      ->add_option("-t,--topic", dump_args_.topic, "Topic to sample (e.g. /tf, /localization/pose)")
      ->required();
    sub->add_option("-o,--output", dump_args_.output_path, "Output file path")->required();
    sub
      ->add_option(
        "-f,--format", dump_args_.format,
        "Output format (tum). When omitted, inferred from the output file extension (e.g. .tum).")
      ->check(CLI::IsMember({kFormatTum}));
    sub->add_option(
      "--ref", dump_args_.ref_frame,
      "Reference frame the trajectory is expressed in. Required for TF topics; "
      "optional for pose / odometry topics.");
    sub->add_option(
      "--of", dump_args_.of_frame,
      "Tracked frame whose trajectory is written. Required for TF topics; "
      "optional for pose / odometry topics.");
    sub->add_flag(
      "-w,--overwrite", dump_args_.overwrite,
      "Replace <output> if it already exists. Without this flag, an "
      "existing output path stops the run.");
    sub->callback([this]() { selected_ = Subcommand::kDump; });
  }

  int run_dump_tf_message(const DumpArgs & args)
  {
    if (
      !args.ref_frame.has_value() || !args.of_frame.has_value() || args.ref_frame->empty() ||
      args.of_frame->empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Topic '%s' is tf2_msgs/msg/TFMessage; both --ref and --of are required.",
        args.topic.c_str());
      return 1;
    }
    if (*args.ref_frame == *args.of_frame) {
      BAGWIZ_LOG_ERROR(
        kLogger, "--ref and --of must be distinct frames; both were '%s'.",
        args.ref_frame->c_str());
      return 1;
    }

    auto reader = io::open_read_or_log(args.input_path, kLogger);
    if (!reader) {
      return 1;
    }

    // /tf_static is one-shot and not a sensible sampling source. The
    // assumption is "1 dynamic + 1 static topic per bag", so the user
    // is expected to point at the dynamic side.
    if (core::is_static_tf_topic(args.topic)) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "Topic '%s' looks like a static TF topic (name ends with 'tf_static'). "
        "Pass the dynamic /tf-style topic instead; static TF in the bag is loaded automatically.",
        args.topic.c_str());
      return 1;
    }

    const auto tf_topics = core::collect_tf_topics(*reader);
    if (tf_topics.empty()) {
      BAGWIZ_LOG_ERROR(kLogger, "Bag has no tf2_msgs/msg/TFMessage topics; nothing to dump.");
      return 1;
    }

    // Walk every TF topic once: feed each contained TransformStamped into the
    // buffer (static or dynamic per topic name) and record the input topic's
    // (frame_id, child_frame_id, stamp_ns) edges so chain-edge filtering below
    // needs no second bag pass. The merge checker refuses contradictory TF.
    tf2::BufferCore tf_buffer{std::chrono::hours(24 * 365)};
    std::vector<core::TfInputEdge> input_edges;
    try {
      core::TfMergeConflictChecker conflict_checker;
      core::TfReplayOutputs outputs;
      outputs.buffer = &tf_buffer;
      outputs.conflict_checker = &conflict_checker;
      outputs.input_topic = args.topic;
      outputs.input_edges = &input_edges;
      core::replay_tf_topics(*reader, tf_topics, outputs);
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
    const auto chain = core::resolve_chain(tf_buffer, *args.of_frame, *args.ref_frame, resolve_tp);
    if (chain.empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "No TF path between '%s' and '%s' in the bag (different connected components, "
        "or one of the frames is absent).",
        args.ref_frame->c_str(), args.of_frame->c_str());
      return 1;
    }
    const auto path_edges = core::chain_to_edges(tf_buffer, chain, resolve_tp);
    const std::vector<std::int64_t> sample_stamps =
      core::collect_path_sample_stamps(input_edges, path_edges);

    if (sample_stamps.empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "No chain edges between '%s' and '%s' are published on '%s'. "
        "The whole chain may live on /tf_static (a fully-static path is rejected by "
        "traj dump — there is no time axis to sample), or the input topic does not "
        "carry the relevant edge.",
        args.ref_frame->c_str(), args.of_frame->c_str(), args.topic.c_str());
      return 1;
    }

    const auto lookup =
      core::lookup_trajectory_at_stamps(tf_buffer, *args.ref_frame, *args.of_frame, sample_stamps);

    if (lookup.poses.empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger, "All %zu sample stamps failed to resolve via lookupTransform. Last reason: %s",
        sample_stamps.size(),
        lookup.last_skip_reason.empty() ? "(none recorded)" : lookup.last_skip_reason.c_str());
      return 1;
    }

    if (!write_tum_file(args.output_path, lookup.poses, kLogger)) {
      return 1;
    }

    BAGWIZ_LOG_INFO(
      kLogger, "Wrote %zu poses (from %zu sample stamps, %" PRId64 " skipped) to %s in %s format",
      lookup.poses.size(), sample_stamps.size(), lookup.skipped, args.output_path.c_str(),
      args.format.c_str());
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
    auto reader = io::open_read_or_log(args.input_path, kLogger);
    if (!reader) {
      return false;
    }
    const auto tf_topics = core::collect_tf_topics(*reader);
    if (tf_topics.empty()) {
      BAGWIZ_LOG_ERROR(
        kLogger,
        "Topic '%s' needs a TF lookup (--ref / --of) but the bag has no "
        "tf2_msgs/msg/TFMessage topics to resolve it.",
        args.topic.c_str());
      return false;
    }
    try {
      // No input_edges output: load every TF topic into the buffer only.
      core::TfMergeConflictChecker conflict_checker;
      core::TfReplayOutputs outputs;
      outputs.buffer = &tf_buffer;
      outputs.conflict_checker = &conflict_checker;
      core::replay_tf_topics(*reader, tf_topics, outputs);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to load TF from the bag: %s", e.what());
      return false;
    }
    return true;
  }

  // Unified pose / odometry trajectory dump.
  //
  // Each output row is the pose of the tracked frame `--of` expressed in the
  // reference frame `--ref`, composed as
  //
  //   T_ref_of = T_ref_header * T_header_body * T_body_of
  //
  // where T_header_body is the message's own pose and the two bridges come
  // from the bag's TF tree (static + dynamic). Frames default per message:
  // `--ref` to header.frame_id (no remap) and, for Odometry, `--of` to
  // child_frame_id (no tracked-side traversal). PoseStamped / PWC do not name
  // their body, so `--of` is accepted as the asserted body frame but never
  // traverses further (the pose already encodes the body); only `--ref`
  // re-expresses them.
  int run_dump_pose_topic(
    const DumpArgs & args, const io::TopicInfo & topic_info, PoseDumpKind kind)
  {
    const bool is_odom = (kind == PoseDumpKind::Odometry);

    if (args.ref_frame.has_value() && args.ref_frame->empty()) {
      BAGWIZ_LOG_ERROR(kLogger, "When set, --ref must be a non-empty frame id.");
      return 1;
    }
    if (args.of_frame.has_value() && args.of_frame->empty()) {
      BAGWIZ_LOG_ERROR(kLogger, "When set, --of must be a non-empty frame id.");
      return 1;
    }

    // A TF lookup can be non-identity only when --ref is set (reference-side
    // bridge) or Odometry has --of set (tracked-side bridge). Pure raw dumps
    // (no flags) need no TF tree at all.
    const bool need_tree = args.ref_frame.has_value() || (is_odom && args.of_frame.has_value());

    tf2::BufferCore tf_buffer{std::chrono::hours(24 * 365)};
    if (!build_dump_tf_tree(args, need_tree, tf_buffer)) {
      return 1;
    }

    auto reader = io::open_read_or_log(args.input_path, kLogger);
    if (!reader) {
      return 1;
    }
    io::ReadFilter filter;
    filter.topics.push_back(args.topic);
    reader->set_filter(filter);

    const auto decoder = open_topic_decoder(*reader, args.topic, kLogger);
    if (!decoder) {
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
      const std::string ref_frame = args.ref_frame.has_value() ? *args.ref_frame : header_frame;
      const std::int64_t ns =
        static_cast<std::int64_t>(sample.pose.header.stamp.sec) * 1'000'000'000LL +
        static_cast<std::int64_t>(sample.pose.header.stamp.nanosec);

      try {
        // Compose T_ref_of = T_ref_header * T_header_body * T_body_of with the
        // two TF bridges resolved at the sample stamp. The tracked-side
        // bridge is engaged for Odometry only (pose topics never traverse:
        // their --of is an asserted body frame, so nullopt is passed).
        poses.push_back(
          core::compose_tf_bridged_sample(
            tf_buffer, ref_frame, header_frame, is_odom ? args.of_frame : std::nullopt,
            sample.child_frame, sample.pose.pose, ns));
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

    if (!write_tum_file(args.output_path, poses, kLogger)) {
      return 1;
    }

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

    auto reader = io::open_read_or_log(args.input_path, kLogger);
    if (!reader) {
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
        "Trajectory format is not set (--format) and the trajectory path '%s' has no usable "
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
      "Trajectory format is not set (--format) and extension '.%s' is not recognized; "
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
    sub->add_option("-i,--input", join_args_.input_path, "Bag path (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    sub
      ->add_option(
        "--traj", join_args_.traj_path,
        "Trajectory file. Format is selected by --format, or inferred from the file extension "
        "when --format is omitted.")
      ->required()
      ->check(CLI::ExistingFile);
    sub
      ->add_option(
        "-t,--topic", join_args_.topic,
        "Topic name to embed the trajectory under. When the topic already exists in <input>, "
        "pass --force to drop its existing messages and replace them.")
      ->required();
    sub->add_option(
      "-o,--output", join_args_.output_path,
      "Output bag path. When omitted, <input> is replaced in place via a sibling tmp directory.");
    sub
      ->add_option(
        "--format", join_args_.format,
        "Trajectory format id. When omitted, inferred from the trajectory file's extension.")
      ->check(CLI::IsMember({kFormatTum}));
    sub
      ->add_option(
        "-m,--msg-type", join_args_.msg_type,
        "ROS message type to publish under <topic>. Only 'tf' (tf2_msgs/msg/TFMessage) is "
        "supported today.")
      ->check(CLI::IsMember({kJoinMsgTypeTf}));
    sub->add_option(
      "--ref", join_args_.ref_frame,
      "Reference frame the trajectory is expressed in; mapped to "
      "TransformStamped.header.frame_id. Required for --msg-type tf.");
    sub->add_option(
      "--of", join_args_.of_frame,
      "Tracked frame whose trajectory is embedded; mapped to "
      "TransformStamped.child_frame_id. Required for --msg-type tf.");
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
      out_transforms.push_back(core::pose_to_transform_stamped(p, *args.ref_frame, *args.of_frame));
      out_stamps_ns.push_back(p.timestamp_ns);
    }
    return true;
  }

  // Execute one full pass: open reader, plan the topic conflict,
  // declare topics on the writer, stream-copy with suppression, append
  // the injected payloads. Used for both in-place and explicit-output
  // modes; the writer factory is parameterised so the rewrite dispatch
  // (core::run_bag_rewrite) can hand in a tmp path. The numbered phases
  // below are the extracted helpers grouped at the top of this file.
  static int execute_join_pass(
    const JoinArgs & args, std::span<const geometry_msgs::msg::TransformStamped> transforms,
    std::span<const std::int64_t> stamps_ns, const io::WriterFactory & open_writer)
  {
    constexpr const char * kExpectedType = "tf2_msgs/msg/TFMessage";

    // 1. Open the input and snapshot its topic list for conflict detection;
    //    the reader's span is invalidated by subsequent operations.
    auto reader = io::open_read_or_log(args.input_path, kLogger);
    if (!reader) {
      return 1;
    }
    const std::vector<io::TopicInfo> input_topics_pre(
      reader->topics().begin(), reader->topics().end());

    // 2. Count existing messages on <topic> and decide the conflict policy.
    const auto existing_count = count_join_topic_messages(*reader, args.topic, args.input_path);
    if (!existing_count.has_value()) {
      return 1;
    }
    const auto decision = core::decide_topic_write(
      input_topics_pre, args.topic, kExpectedType, *existing_count, args.force);
    if (!join_topic_decision_proceeds(decision)) {
      return 1;
    }

    // 3. Open the writer, then backfill schemas. Deferring the schema load
    //    avoids opening shard 0 for bags that abort early due to a topic
    //    conflict; the post-backfill snapshot is what lets the output writer
    //    receive embedded schemas.
    auto writer = io::open_write_or_log(open_writer, kLogger);
    if (!writer) {
      return 1;
    }
    reader->populate_schemas();
    const std::vector<io::TopicInfo> input_topics(reader->topics().begin(), reader->topics().end());

    // 4. Declare the input topics (plus <topic> itself when it is new).
    if (!declare_join_pass_topics(*writer, input_topics, decision.action, args.topic)) {
      return 1;
    }

    // 5. Serialize the trajectory up front so the copy can interleave it.
    auto injected_messages = build_join_tf_messages(args.topic, transforms, stamps_ns);
    if (!injected_messages.has_value()) {
      return 1;
    }

    // 6. Stream-copy through the merging writer, suppressing <topic>'s existing
    //    payloads on --force. Each trajectory sample lands at its own timestamp
    //    rather than after every copied message.
    core::InjectingWriter ordered(*writer, std::move(*injected_messages));
    const auto counts =
      copy_join_pass_messages(*reader, ordered, decision.action, args.topic, args.input_path);
    if (!counts.has_value()) {
      return 1;
    }
    // Flush samples stamped after the bag's last message.
    ordered.close();
    const std::uint64_t injected = ordered.injected_count();

    // 7. Close and summarise.
    if (!io::close_writer_or_log(*writer, kLogger)) {
      return 1;
    }
    BAGWIZ_LOG_INFO(
      kLogger,
      "traj join: copied %" PRIu64 " message(s), suppressed %" PRIu64 ", injected %" PRIu64
      " TFMessage(s) on '%s'.",
      counts->copied, counts->suppressed, injected, args.topic.c_str());
    return 0;
  }

  int run_join()
  {
    const auto & args = join_args_;

    // 1. Validate type-specific arg constraints. Today only --msg-type tf
    //    is supported, and it requires --ref / --of as distinct non-empty
    //    frame ids.
    if (args.msg_type == kJoinMsgTypeTf) {
      if (
        !args.ref_frame.has_value() || !args.of_frame.has_value() || args.ref_frame->empty() ||
        args.of_frame->empty()) {
        BAGWIZ_LOG_ERROR(
          kLogger,
          "--msg-type tf requires both --ref (parent frame_id) and --of (child_frame_id).");
        return 1;
      }
      if (*args.ref_frame == *args.of_frame) {
        BAGWIZ_LOG_ERROR(
          kLogger, "--ref and --of must be distinct frames; both were '%s'.",
          args.ref_frame->c_str());
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

    // 4. -o vs in-place dispatch, shared with the other rewrite-style
    //    commands: -o writes a fresh bag (format/layout resolved from the
    //    output path's extension) and leaves <input> untouched; otherwise
    //    <input> is rewritten atomically via a sibling tmp, preserving its
    //    storage identity (Format::Auto would misread the tmp's synthetic
    //    suffix and silently convert db3 inputs to mcap on swap).
    core::BagRewriteOptions rewrite_opts;
    rewrite_opts.logger = kLogger;
    rewrite_opts.format_unknown_error =
      "traj join: could not detect storage format of input bag '%s'.";
    rewrite_opts.pass_failed_error = "traj join: pass failed; aborting in-place swap";
    return core::run_bag_rewrite(
      args.input_path, args.output_path, args.overwrite, rewrite_opts,
      [&](const io::WriterFactory & open_writer) {
        return execute_join_pass(args, transforms, stamps_ns, open_writer);
      });
  }
};

BAGWIZ_REGISTER_COMMAND(TrajCommand)

}  // namespace bagwiz::commands
