// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tf/tf_trajectory_sample.hpp"

#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/tf/tf_chain.hpp"
#include "bagwiz/core/tf/tf_value_extract.hpp"

#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <algorithm>
#include <chrono>
#include <exception>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
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

TfMessageTrajectoryResult sample_tf_message_trajectory(
  const std::filesystem::path & input_path, const io::TopicInfo & topic,
  const std::string & ref_frame, const std::string & of_frame, tf2::BufferCore & buffer)
{
  using Failure = TfMessageTrajectoryResult::Failure;
  TfMessageTrajectoryResult out;

  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(input_path);
  } catch (const std::exception & e) {
    out.failure = Failure::kOpenBag;
    out.failure_detail = e.what();
    return out;
  }
  reader->populate_schemas();
  io::ReadFilter filter;
  filter.topics = {topic.name};
  reader->set_filter(filter);

  auto open = decoder::open_decoder(topic);
  if (!open.ok()) {
    out.failure = Failure::kOpenDecoder;
    out.failure_detail = open.error;
    return out;
  }

  std::vector<TfInputEdge> input_edges;
  io::RawMessage raw;
  try {
    while (reader->next(raw)) {
      if (raw.topic->name != topic.name) {
        continue;
      }
      const auto decoded = open.decoder->decode(raw.payload);
      if (!decoded.ok()) {
        out.failure = Failure::kDecode;
        out.failure_detail = decoded.error;
        return out;
      }
      for (const auto & t : extract_tf_message(*decoded.value)) {
        buffer.setTransform(t, "bagwiz", /*is_static=*/false);
        const std::int64_t ns = static_cast<std::int64_t>(t.header.stamp.sec) * 1'000'000'000LL +
                                static_cast<std::int64_t>(t.header.stamp.nanosec);
        input_edges.push_back({t.header.frame_id, t.child_frame_id, ns});
      }
    }
  } catch (const std::exception & e) {
    out.failure = Failure::kRead;
    out.failure_detail = e.what();
    return out;
  }

  if (input_edges.empty()) {
    out.failure = Failure::kNoTransforms;
    return out;
  }

  // Resolve the chain using the first published edge's stamp: parent/child
  // linkage in tf2 is fixed for a frame's whole life, so any populated stamp
  // works — this just needs to land inside what was just set above.
  const tf2::TimePoint resolve_tp{std::chrono::nanoseconds(input_edges.front().stamp_ns)};
  const auto chain = resolve_chain(buffer, of_frame, ref_frame, resolve_tp);
  if (chain.empty()) {
    out.failure = Failure::kNoPath;
    return out;
  }
  const auto path_edges = chain_to_edges(buffer, chain, resolve_tp);
  const std::vector<std::int64_t> sample_stamps =
    collect_path_sample_stamps(input_edges, path_edges);
  if (sample_stamps.empty()) {
    out.failure = Failure::kNoPathStamps;
    return out;
  }

  const auto lookup = lookup_trajectory_at_stamps(buffer, ref_frame, of_frame, sample_stamps);
  out.poses = std::move(lookup.poses);
  out.skipped = lookup.skipped;
  out.last_skip_reason = std::move(lookup.last_skip_reason);
  out.sample_stamps = sample_stamps.size();
  return out;
}

}  // namespace bagwiz::core
