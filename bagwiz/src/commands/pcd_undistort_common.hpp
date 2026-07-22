// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__PCD_UNDISTORT_COMMON_HPP_
#define COMMANDS__PCD_UNDISTORT_COMMON_HPP_

#include "bagwiz/core/cdr_walker/value.hpp"
#include "bagwiz/core/pointcloud/pointcloud2.hpp"
#include "bagwiz/core/tf/trajectory.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <tf2/buffer_core.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

// Internals of `pcd undistort`, split out of pcd_undistort.cpp so the
// validation, trajectory-building, first-cloud peek, and extrinsic-resolution
// units can be unit-tested without driving the full command. CLI-internal:
// this header lives with the command sources and is not installed.
namespace bagwiz::commands
{

// The pose topic types the trajectory builder can consume.
[[nodiscard]] bool is_supported_pose_topic_type(const std::string & type);

// The 3 pose-shaped topic types that carry their own body pose directly
// (as opposed to TFMessage, which is a set of independent edges).
enum class PoseComposeKind { kOdometry, kPoseStamped, kPoseWithCovarianceStamped };

// Map a pose topic type to its composition kind. The caller validates the
// type first (is_supported_pose_topic_type), so anything left maps to
// kPoseWithCovarianceStamped.
[[nodiscard]] PoseComposeKind pose_compose_kind(const std::string & type);

// One decoded sample from a pose / odometry input topic. `child_frame` is
// only set for Odometry.
struct PoseSample
{
  geometry_msgs::msg::PoseStamped pose;
  std::string child_frame;
};

// Extract one sample from a decoded message; returns false when the payload
// does not parse (the trajectory builder tolerates that as a skipped sample).
[[nodiscard]] bool decode_pose_sample(
  PoseComposeKind kind, const core::cdr_walker::Value & value, PoseSample & out);

// Validate the pose topic (present, of a supported type) and every --pcd
// topic (present, PointCloud2) against the bag's topic list, logging the
// command's usual errors to `logger` on the first failure. Returns the pose
// topic's TopicInfo (aliasing the reader's internal list, valid until the
// reader is destroyed), or nullptr after logging when validation fails.
[[nodiscard]] const io::TopicInfo * validate_undistort_topics(
  const io::BagReader & reader, const std::string & pose_topic,
  const std::vector<std::string> & pcd_topics, const std::filesystem::path & bag_path,
  const char * logger);

// Outcome of building the --of -> --ref trajectory in Pass 1.
struct TrajectoryBuildResult
{
  std::vector<core::TrajectoryPose> trajectory;
  std::string error;  // empty on success

  [[nodiscard]] bool ok() const { return error.empty() && !trajectory.empty(); }
};

// Pass 1: load the bag's static TF into `buffer` (kept by the caller for the
// extrinsic resolution that follows), build the --of -> --ref trajectory from
// the pose topic — TFMessage edges, or pose / odometry samples composed with
// static-TF bridges — sorted by stamp. Logs the command's errors to `logger`;
// on failure returns with !ok() and `error` set.
[[nodiscard]] TrajectoryBuildResult build_sorted_of_ref_trajectory(
  const std::filesystem::path & input_path, const io::TopicInfo & pose_ti, const std::string & ref,
  const std::string & of, tf2::BufferCore & buffer, const char * logger);

// find_point_time_field only reads `.fields`, so a header-only peek (no point
// data copy) is enough to tell whether a --pcd topic has a usable per-point
// time field. A field that is present by name but whose declared offset runs
// past `point_step` is treated the same as an absent field: deskew_pointcloud2
// applies the identical bounds check (its own `fits()` guard) and silently
// falls back to "no usable time" rather than erroring, which would otherwise
// let an out-of-bounds field slip past this upfront, required-time-field
// check and get passed through un-deskewed with no warning.
[[nodiscard]] bool cloud_has_usable_point_time(
  const std::vector<core::pointcloud::PointField> & fields, std::uint32_t point_step);

// One --pcd topic's peeked state: its first cloud's frame and whether that
// cloud carries a usable per-point time field.
struct PcdTopicState
{
  std::string frame_id;
  bool has_time = false;
};

// Peek each --pcd topic's first cloud (frame_id + time field) with one
// filtered reader pass. Returns nullopt after logging on open / header-parse
// / read failure. A topic whose first message never arrives is simply absent
// from the returned map; validate_pcd_topic_states rejects that case.
[[nodiscard]] std::optional<std::unordered_map<std::string, PcdTopicState>> peek_pcd_topic_states(
  const std::filesystem::path & input_path, const std::vector<std::string> & pcd_topics,
  const char * logger);

// Require every --pcd topic to have a peeked state carrying a usable
// per-point time field, logging the command's errors to `logger`.
[[nodiscard]] bool validate_pcd_topic_states(
  const std::vector<std::string> & pcd_topics,
  const std::unordered_map<std::string, PcdTopicState> & states, const char * logger);

// Each --pcd topic's resolved extrinsic E = T_of_cloud; a nullopt entry means
// the cloud frame already is --of (no transform needed).
using ExtrinsicMap = std::unordered_map<std::string, std::optional<geometry_msgs::msg::Transform>>;

// Resolve each --pcd topic's extrinsic from the static TF tree (buffer loaded
// by build_sorted_of_ref_trajectory), logging the command's errors to
// `logger`. Returns nullopt on failure.
[[nodiscard]] std::optional<ExtrinsicMap> resolve_pcd_extrinsics(
  const tf2::BufferCore & buffer, const std::string & of,
  const std::vector<std::string> & pcd_topics,
  const std::unordered_map<std::string, PcdTopicState> & states, const char * logger);

// Worker count for the parallel pass: --threads, clamped to the hardware
// concurrency when that is known. A non-positive value (0 = "auto") resolves
// to the hardware concurrency (1 when unknown); the CLI default of 8 is
// applied by the caller before this function sees the value.
[[nodiscard]] int resolve_num_threads(int requested, unsigned int hardware);

}  // namespace bagwiz::commands

#endif  // COMMANDS__PCD_UNDISTORT_COMMON_HPP_
