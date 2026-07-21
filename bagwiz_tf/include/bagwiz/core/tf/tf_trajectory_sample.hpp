// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__TF__TF_TRAJECTORY_SAMPLE_HPP_
#define BAGWIZ__CORE__TF__TF_TRAJECTORY_SAMPLE_HPP_

#include "bagwiz/core/tf/tf_buffer_loader.hpp"
#include "bagwiz/core/tf/trajectory.hpp"

#include <tf2/buffer_core.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace bagwiz::core
{

// Collect the stamps at which a resolved TF path is actually published on the
// input topic. `input_edges` are the (frame_id, child_frame_id, stamp) edges
// observed on the input topic (TfReplayOutputs::input_edges); `path_edges`
// are the (parent, child) edges of the resolved --of -> --ref chain
// (chain_to_edges). An input edge contributes its stamp when its
// (frame_id, child_frame_id) equals a path edge's (parent, child) — the
// orientation must match exactly. The result is sorted ascending and
// de-duplicated; empty when no input edge lies on the path.
[[nodiscard]] std::vector<std::int64_t> collect_path_sample_stamps(
  std::span<const TfInputEdge> input_edges,
  const std::vector<std::pair<std::string, std::string>> & path_edges);

// Outcome of lookup_trajectory_at_stamps. `poses` holds one entry per stamp
// whose lookup resolved, in input-stamp order; `skipped` counts the stamps
// whose lookup threw tf2::TransformException, and `last_skip_reason` carries
// the last such exception's what() (empty when nothing was skipped).
struct TrajectoryLookupResult
{
  std::vector<TrajectoryPose> poses;
  std::int64_t skipped = 0;
  std::string last_skip_reason;
};

// Resolve the pose of `of_frame` expressed in `ref_frame` at each stamp via
// lookupTransform(target=ref_frame, source=of_frame, stamp). Stamps whose
// lookup throws tf2::TransformException are skipped (counted, with the last
// reason recorded); the caller decides whether an empty `poses` is fatal.
[[nodiscard]] TrajectoryLookupResult lookup_trajectory_at_stamps(
  const tf2::BufferCore & buffer, const std::string & ref_frame, const std::string & of_frame,
  std::span<const std::int64_t> stamps_ns);

// Compose one pose / odometry sample into its output trajectory pose
//
//   T_ref_of = T_ref_header * T_header_body * T_body_of
//
// (see compose_trajectory_pose, which performs the actual composition),
// resolving the two TF bridges against `buffer` at `stamp_ns`:
//
//   * reference-side bridge: when `ref_frame` differs from `header_frame`,
//     lookupTransform(target=ref_frame, source=header_frame); identity
//     (no lookup) when the two are equal.
//   * tracked-side bridge: when `of_frame` is engaged and differs from
//     `child_frame`, lookupTransform(target=child_frame, source=of_frame)
//     (e.g. Odometry child_frame_id -> sensor through static TF); identity
//     when `of_frame` is std::nullopt (pose topics never traverse — their
//     --of is only an asserted body frame) or equals `child_frame`.
//
// The returned pose carries `stamp_ns`. Throws tf2::TransformException when a
// needed lookup fails; the caller counts that as a skipped sample.
[[nodiscard]] TrajectoryPose compose_tf_bridged_sample(
  const tf2::BufferCore & buffer, const std::string & ref_frame, const std::string & header_frame,
  const std::optional<std::string> & of_frame, const std::string & child_frame,
  const geometry_msgs::msg::Pose & body_pose, std::int64_t stamp_ns);

// Outcome of sample_tf_message_trajectory(). Failures are reported as a
// structured `failure` + `failure_detail` pair rather than a prose error
// string, so each caller keeps its own command-level wording (flag names,
// topic context) while the replay/lookup logic stays shared.
struct TfMessageTrajectoryResult
{
  // Stage the scan reached before it failed; kNone on success.
  enum class Failure {
    kNone,
    kOpenBag,       // the bag could not be reopened
    kOpenDecoder,   // no decoder could be created for the topic
    kDecode,        // a message on the topic failed to decode
    kRead,          // storage-level read error
    kNoTransforms,  // the topic carried no TransformStamped entries
    kNoPath,        // no TF path from of_frame to ref_frame
    kNoPathStamps,  // the path resolves via static TF only: none of its edges
                    // is published on the topic, so there is no time axis
  };

  std::vector<TrajectoryPose> poses;  // resolved samples, in input-stamp order
  std::int64_t skipped = 0;           // sample stamps whose lookup threw
  std::string last_skip_reason;       // the last skip's exception what()
  std::size_t sample_stamps = 0;      // stamps considered (poses + skipped)
  Failure failure = Failure::kNone;
  std::string failure_detail;  // exception / decoder message behind `failure`
};

// Build the of_frame -> ref_frame trajectory from a TFMessage pose topic:
// replay the topic's transforms into `buffer` as dynamic edges (the caller
// has pre-loaded the bag's static TF), resolve the chain, sample it at the
// stamps the chain's edges are actually published on this topic, and look up
// each sample. Stamps whose lookup throws are skipped and counted; whether an
// empty `poses` is fatal is the caller's decision (see `sample_stamps` /
// `last_skip_reason`).
[[nodiscard]] TfMessageTrajectoryResult sample_tf_message_trajectory(
  const std::filesystem::path & input_path, const io::TopicInfo & topic,
  const std::string & ref_frame, const std::string & of_frame, tf2::BufferCore & buffer);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__TF__TF_TRAJECTORY_SAMPLE_HPP_
