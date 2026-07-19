// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tf/tf_trajectory_sample.hpp"

#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <algorithm>
#include <chrono>
#include <string>
#include <unordered_set>
#include <vector>

namespace bagwiz::core
{

std::vector<std::int64_t> collect_path_sample_stamps(
  std::span<const TfInputEdge> input_edges,
  const std::vector<std::pair<std::string, std::string>> & path_edges)
{
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

  std::sort(sample_stamps.begin(), sample_stamps.end());
  sample_stamps.erase(std::unique(sample_stamps.begin(), sample_stamps.end()), sample_stamps.end());
  return sample_stamps;
}

TrajectoryLookupResult lookup_trajectory_at_stamps(
  const tf2::BufferCore & buffer, const std::string & ref_frame, const std::string & of_frame,
  std::span<const std::int64_t> stamps_ns)
{
  TrajectoryLookupResult result;
  result.poses.reserve(stamps_ns.size());
  for (const std::int64_t ns : stamps_ns) {
    const tf2::TimePoint tp{std::chrono::nanoseconds(ns)};
    try {
      const auto tf = buffer.lookupTransform(ref_frame, of_frame, tp);
      TrajectoryPose p;
      p.timestamp_ns = ns;
      p.tx = tf.transform.translation.x;
      p.ty = tf.transform.translation.y;
      p.tz = tf.transform.translation.z;
      p.qx = tf.transform.rotation.x;
      p.qy = tf.transform.rotation.y;
      p.qz = tf.transform.rotation.z;
      p.qw = tf.transform.rotation.w;
      result.poses.push_back(p);
    } catch (const tf2::TransformException & e) {
      ++result.skipped;
      result.last_skip_reason = e.what();
    }
  }
  return result;
}

TrajectoryPose compose_tf_bridged_sample(
  const tf2::BufferCore & buffer, const std::string & ref_frame, const std::string & header_frame,
  const std::optional<std::string> & of_frame, const std::string & child_frame,
  const geometry_msgs::msg::Pose & body_pose, std::int64_t stamp_ns)
{
  const tf2::TimePoint tp{std::chrono::nanoseconds(stamp_ns)};

  // Reference-side bridge: re-express the result into --ref. Identity
  // (no lookup) when --ref already equals header.frame_id.
  std::optional<geometry_msgs::msg::Transform> from_header;
  if (ref_frame != header_frame) {
    from_header = buffer.lookupTransform(ref_frame, header_frame, tp).transform;
  }
  // Tracked-side bridge: walk body/child -> --of via the TF tree (e.g.
  // base_link -> sensor through static TF). Identity when --of is not
  // engaged or already equals the body frame.
  std::optional<geometry_msgs::msg::Transform> body_to;
  if (of_frame.has_value() && *of_frame != child_frame) {
    body_to = buffer.lookupTransform(child_frame, *of_frame, tp).transform;
  }

  const auto out_pose = compose_trajectory_pose(from_header, body_pose, body_to);
  TrajectoryPose p;
  p.timestamp_ns = stamp_ns;
  p.tx = out_pose.position.x;
  p.ty = out_pose.position.y;
  p.tz = out_pose.position.z;
  p.qx = out_pose.orientation.x;
  p.qy = out_pose.orientation.y;
  p.qz = out_pose.orientation.z;
  p.qw = out_pose.orientation.w;
  return p;
}

}  // namespace bagwiz::core
