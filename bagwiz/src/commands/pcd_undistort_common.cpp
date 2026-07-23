// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "pcd_undistort_common.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/pointcloud/point_time.hpp"
#include "bagwiz/core/pointcloud/static_extrinsic.hpp"
#include "bagwiz/core/tf/tf_buffer_loader.hpp"
#include "bagwiz/core/tf/tf_trajectory_sample.hpp"
#include "bagwiz/core/tf/tf_value_extract.hpp"
#include "bagwiz/io/bag_open.hpp"
#include "bagwiz/io/topics.hpp"

#include <algorithm>
#include <cstdint>
#include <exception>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace bagwiz::commands
{
namespace
{

constexpr const char * kPointCloud2Type = "sensor_msgs/msg/PointCloud2";
constexpr const char * kTfMessageType = "tf2_msgs/msg/TFMessage";
constexpr const char * kOdometryType = "nav_msgs/msg/Odometry";
constexpr const char * kPoseStampedType = "geometry_msgs/msg/PoseStamped";
constexpr const char * kPoseWithCovarianceStampedType =
  "geometry_msgs/msg/PoseWithCovarianceStamped";

// TFMessage pose topic: delegate to the shared bagwiz_tf sampler
// (core::sample_tf_message_trajectory), which replays the topic's transforms
// into `buffer` as dynamic edges (tf_static is already loaded there),
// resolves the --ref -> --of chain, and samples it at every stamp the chain's
// edges are actually published on `pose_topic`. The switch below only maps
// the sampler's structured failure onto this command's wording, so log texts
// and exit behavior stay unchanged.
TrajectoryBuildResult build_trajectory_from_tf_message(
  const std::filesystem::path & input_path, const io::TopicInfo & pose_ti, const std::string & ref,
  const std::string & of, tf2::BufferCore & buffer)
{
  TrajectoryBuildResult out;
  const auto sampled = core::sample_tf_message_trajectory(input_path, pose_ti, ref, of, buffer);
  using Failure = core::TfMessageTrajectoryResult::Failure;
  switch (sampled.failure) {
    case Failure::kNone:
      break;
    case Failure::kOpenBag:
      out.error = "failed to reopen bag for pose topic: " + sampled.failure_detail;
      return out;
    case Failure::kOpenDecoder:
      out.error =
        "could not open decoder for pose topic '" + pose_ti.name + "': " + sampled.failure_detail;
      return out;
    case Failure::kDecode:
      out.error = "failed to decode message on '" + pose_ti.name + "': " + sampled.failure_detail;
      return out;
    case Failure::kRead:
      out.error = "error reading pose topic '" + pose_ti.name + "': " + sampled.failure_detail;
      return out;
    case Failure::kNoTransforms:
      out.error = "pose topic '" + pose_ti.name + "' carried no TransformStamped entries";
      return out;
    case Failure::kNoPath:
      out.error = "no TF path from --of '" + of + "' to --ref '" + ref + "' (checked '" +
                  pose_ti.name + "' + the bag's static TF)";
      return out;
    case Failure::kNoPathStamps:
      out.error = "--of '" + of + "' -> --ref '" + ref +
                  "' resolves via static TF, but none of the edges on that path are published on "
                  "pose topic '" +
                  pose_ti.name + "'";
      return out;
  }

  if (sampled.poses.empty()) {
    out.error = "all " + std::to_string(sampled.sample_stamps) +
                " sample stamp(s) failed to resolve via lookupTransform; last reason: " +
                (sampled.last_skip_reason.empty() ? "(none)" : sampled.last_skip_reason);
    return out;
  }
  out.trajectory = std::move(sampled.poses);
  return out;
}

// Odometry / PoseStamped / PoseWithCovarianceStamped pose topic: for each
// message, compose T_ref_of = T_ref_header * T_header_body * T_body_of,
// bridging into --ref / --of via the bag's static TF when the message's own
// frames do not already match. Mirrors the traj dump pose-topic composition
// (core::compose_tf_bridged_sample in bagwiz_tf), except an unresolvable
// bridge is fatal here rather than a per-sample skip: pcd undistort's TF is
// static-only, so a failure is a configuration problem, not transient sensor
// noise.
TrajectoryBuildResult build_trajectory_from_pose_topic(
  const std::filesystem::path & input_path, const io::TopicInfo & pose_ti, PoseComposeKind kind,
  const std::string & ref, const std::string & of, tf2::BufferCore & buffer)
{
  TrajectoryBuildResult out;
  const bool is_odom = (kind == PoseComposeKind::kOdometry);

  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(input_path);
  } catch (const std::exception & e) {
    out.error = std::string("failed to reopen bag for pose topic: ") + e.what();
    return out;
  }
  reader->populate_schemas();
  io::ReadFilter filter;
  filter.topics = {pose_ti.name};
  reader->set_filter(filter);

  auto open = core::decoder::open_decoder(pose_ti);
  if (!open.ok()) {
    out.error = "could not open decoder for pose topic '" + pose_ti.name + "': " + open.error;
    return out;
  }

  io::RawMessage raw;
  try {
    while (reader->next(raw)) {
      if (raw.topic->name != pose_ti.name) {
        continue;
      }
      const auto decoded = open.decoder->decode(raw.payload);
      if (!decoded.ok()) {
        out.error = "failed to decode message on '" + pose_ti.name + "': " + decoded.error;
        return out;
      }
      PoseSample sample;
      if (!decode_pose_sample(kind, *decoded.value, sample)) {
        continue;  // unparsable sample; tolerated like traj dump's skip
      }
      if (sample.pose.header.frame_id.empty()) {
        out.error = "pose topic '" + pose_ti.name + "': message has empty header.frame_id";
        return out;
      }
      if (is_odom && sample.child_frame.empty()) {
        out.error = "pose topic '" + pose_ti.name + "': Odometry message has empty child_frame_id";
        return out;
      }

      const std::string & header_frame = sample.pose.header.frame_id;
      const std::int64_t ns =
        static_cast<std::int64_t>(sample.pose.header.stamp.sec) * 1'000'000'000LL +
        static_cast<std::int64_t>(sample.pose.header.stamp.nanosec);

      std::optional<geometry_msgs::msg::Transform> from_header;
      if (ref != header_frame) {
        const auto resolved = core::pointcloud::resolve_static_extrinsic(buffer, ref, header_frame);
        if (!resolved.missing.empty()) {
          out.error = "--ref '" + ref + "' has no static TF chain to pose topic '" + pose_ti.name +
                      "'s frame '" + header_frame + "'";
          return out;
        }
        if (!resolved.ok()) {
          out.error = "--ref '" + ref + "' -> '" + header_frame +
                      "' TF lookup failed: " + resolved.lookup_error;
          return out;
        }
        from_header = resolved.transform.transform;
      }
      std::optional<geometry_msgs::msg::Transform> body_to;
      if (is_odom && of != sample.child_frame) {
        const auto resolved =
          core::pointcloud::resolve_static_extrinsic(buffer, sample.child_frame, of);
        if (!resolved.missing.empty()) {
          out.error = "--of '" + of + "' has no static TF chain from Odometry child frame '" +
                      sample.child_frame + "'";
          return out;
        }
        if (!resolved.ok()) {
          out.error = "'" + sample.child_frame + "' -> --of '" + of +
                      "' TF lookup failed: " + resolved.lookup_error;
          return out;
        }
        body_to = resolved.transform.transform;
      }

      const auto composed = core::compose_trajectory_pose(from_header, sample.pose.pose, body_to);
      core::TrajectoryPose p;
      p.timestamp_ns = ns;
      p.tx = composed.position.x;
      p.ty = composed.position.y;
      p.tz = composed.position.z;
      p.qx = composed.orientation.x;
      p.qy = composed.orientation.y;
      p.qz = composed.orientation.z;
      p.qw = composed.orientation.w;
      out.trajectory.push_back(p);
    }
  } catch (const std::exception & e) {
    out.error = "error reading pose topic '" + pose_ti.name + "': " + e.what();
    return out;
  }

  if (out.trajectory.empty()) {
    out.error = "no poses decoded from pose topic '" + pose_ti.name + "'";
  }
  return out;
}

}  // namespace

bool is_supported_pose_topic_type(const std::string & type)
{
  return type == kTfMessageType || type == kOdometryType || type == kPoseStampedType ||
         type == kPoseWithCovarianceStampedType;
}

PoseComposeKind pose_compose_kind(const std::string & type)
{
  if (type == kOdometryType) {
    return PoseComposeKind::kOdometry;
  }
  if (type == kPoseStampedType) {
    return PoseComposeKind::kPoseStamped;
  }
  return PoseComposeKind::kPoseWithCovarianceStamped;  // caller validated the type
}

bool decode_pose_sample(
  PoseComposeKind kind, const core::cdr_walker::Value & value, PoseSample & out)
{
  switch (kind) {
    case PoseComposeKind::kPoseStamped: {
      const auto ps = core::extract_pose_stamped_message(value);
      if (!ps.has_value()) {
        return false;
      }
      out.pose = *ps;
      return true;
    }
    case PoseComposeKind::kPoseWithCovarianceStamped: {
      const auto pwc = core::extract_pose_with_covariance_stamped_message(value);
      if (!pwc.has_value()) {
        return false;
      }
      out.pose.header = pwc->header;
      out.pose.pose = pwc->pose.pose;
      return true;
    }
    case PoseComposeKind::kOdometry: {
      const auto odom = core::extract_odometry_message(value);
      if (!odom.has_value()) {
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

const io::TopicInfo * validate_undistort_topics(
  const io::BagReader & reader, const std::string & pose_topic,
  const std::vector<std::string> & pcd_topics, const std::filesystem::path & bag_path,
  const char * logger)
{
  const io::TopicInfo * pose_ti = io::find_topic_or_log(reader, pose_topic, bag_path, logger);
  if (pose_ti == nullptr) {
    return nullptr;
  }
  if (!is_supported_pose_topic_type(pose_ti->type)) {
    BAGWIZ_LOG_ERROR(
      logger, "Topic '%s' has unsupported type '%s'. Supported: %s, %s, %s, %s.",
      pose_topic.c_str(), pose_ti->type.c_str(), kTfMessageType, kOdometryType, kPoseStampedType,
      kPoseWithCovarianceStampedType);
    return nullptr;
  }
  for (const auto & topic : pcd_topics) {
    const io::TopicInfo * info = io::find_topic_or_log(reader, topic, bag_path, logger);
    if (info == nullptr) {
      return nullptr;
    }
    if (info->type != kPointCloud2Type) {
      BAGWIZ_LOG_ERROR(
        logger, "Topic '%s' is %s, expected %s", topic.c_str(), info->type.c_str(),
        kPointCloud2Type);
      return nullptr;
    }
  }
  return pose_ti;
}

TrajectoryBuildResult build_sorted_of_ref_trajectory(
  const std::filesystem::path & input_path, const io::TopicInfo & pose_ti, const std::string & ref,
  const std::string & of, tf2::BufferCore & buffer, const char * logger)
{
  TrajectoryBuildResult out;
  if (const auto error = core::load_static_tf_buffer(input_path, buffer); error.has_value()) {
    // load_static_tf_buffer is a shared, caller-neutral helper (it names no
    // command's flags), so its detail is always safe to forward here.
    BAGWIZ_LOG_ERROR(
      logger,
      "pcd undistort: could not load the bag's static TF (needed to resolve --ref '%s' / --of "
      "'%s' and any --pcd topic's sensor extrinsic); detail: %s",
      ref.c_str(), of.c_str(), error->c_str());
    out.error = *error;
    return out;
  }

  out = (pose_ti.type == kTfMessageType)
          ? build_trajectory_from_tf_message(input_path, pose_ti, ref, of, buffer)
          : build_trajectory_from_pose_topic(
              input_path, pose_ti, pose_compose_kind(pose_ti.type), ref, of, buffer);
  if (!out.ok()) {
    BAGWIZ_LOG_ERROR(
      logger, "pcd undistort: could not resolve --of '%s' -> --ref '%s' from pose topic '%s': %s",
      of.c_str(), ref.c_str(), pose_ti.name.c_str(), out.error.c_str());
    return out;
  }
  std::sort(out.trajectory.begin(), out.trajectory.end(), [](const auto & a, const auto & b) {
    return a.timestamp_ns < b.timestamp_ns;
  });
  return out;
}

bool cloud_has_usable_point_time(
  const std::vector<core::pointcloud::PointField> & fields, std::uint32_t point_step)
{
  core::pointcloud::PointCloud2 shim;
  shim.fields = fields;
  const auto field = core::pointcloud::find_point_time_field(shim);
  if (!field.has_value()) {
    return false;
  }
  return static_cast<std::size_t>(field->offset) +
           core::pointcloud::datatype_size(field->datatype) <=
         point_step;
}

std::optional<std::unordered_map<std::string, PcdTopicState>> peek_pcd_topic_states(
  const std::filesystem::path & input_path, const std::vector<std::string> & pcd_topics,
  const char * logger)
{
  std::unordered_map<std::string, PcdTopicState> states;
  std::unique_ptr<io::BagReader> preader;
  try {
    preader = io::open_read(input_path);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(logger, "Failed to reopen %s: %s", input_path.c_str(), e.what());
    return std::nullopt;
  }
  io::ReadFilter filter;
  filter.topics = pcd_topics;
  preader->set_filter(filter);
  std::unordered_set<std::string> pending(pcd_topics.begin(), pcd_topics.end());
  io::RawMessage raw;
  try {
    while (!pending.empty() && preader->next(raw)) {
      const auto it = pending.find(raw.topic->name);
      if (it == pending.end()) {
        continue;  // already peeked this topic's first message
      }
      const auto header = core::pointcloud::parse_pointcloud2_header(raw.payload);
      if (!header.ok()) {
        BAGWIZ_LOG_ERROR(
          logger, "pcd undistort: could not parse the first message on --pcd topic '%s': %s",
          raw.topic->name.c_str(), header.error.c_str());
        return std::nullopt;
      }
      PcdTopicState st;
      st.frame_id = header.header->frame_id;
      st.has_time = cloud_has_usable_point_time(header.header->fields, header.header->point_step);
      states.emplace(raw.topic->name, st);
      pending.erase(it);
    }
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(logger, "read error peeking --pcd topics: %s", e.what());
    return std::nullopt;
  }
  return states;
}

bool validate_pcd_topic_states(
  const std::vector<std::string> & pcd_topics,
  const std::unordered_map<std::string, PcdTopicState> & states, const char * logger)
{
  for (const auto & topic : pcd_topics) {
    const auto it = states.find(topic);
    if (it == states.end()) {
      BAGWIZ_LOG_ERROR(
        logger, "pcd undistort: --pcd topic '%s' has no decodable PointCloud2 message",
        topic.c_str());
      return false;
    }
    if (!it->second.has_time) {
      BAGWIZ_LOG_ERROR(
        logger,
        "pcd undistort: --pcd topic '%s' has no per-point time field (checked t / time / "
        "time_stamp / timestamp); pcd undistort requires per-point time to deskew",
        topic.c_str());
      return false;
    }
  }
  return true;
}

std::optional<ExtrinsicMap> resolve_pcd_extrinsics(
  const tf2::BufferCore & buffer, const std::string & of,
  const std::vector<std::string> & pcd_topics,
  const std::unordered_map<std::string, PcdTopicState> & states, const char * logger)
{
  ExtrinsicMap extrinsics;
  for (const auto & topic : pcd_topics) {
    const std::string & frame_id = states.at(topic).frame_id;
    std::optional<geometry_msgs::msg::Transform> extrinsic;
    if (frame_id != of) {
      const auto resolved = core::pointcloud::resolve_static_extrinsic(buffer, of, frame_id);
      if (!resolved.missing.empty()) {
        BAGWIZ_LOG_ERROR(
          logger, "pcd undistort: --of '%s' has no static TF chain to --pcd topic '%s' frame '%s'",
          of.c_str(), topic.c_str(), frame_id.c_str());
        return std::nullopt;
      }
      if (!resolved.ok()) {
        BAGWIZ_LOG_ERROR(
          logger, "pcd undistort: --of '%s' -> --pcd topic '%s' frame '%s' TF lookup failed: %s",
          of.c_str(), topic.c_str(), frame_id.c_str(), resolved.lookup_error.c_str());
        return std::nullopt;
      }
      extrinsic = resolved.transform.transform;
    }
    extrinsics.emplace(topic, extrinsic);
  }
  return extrinsics;
}

}  // namespace bagwiz::commands
