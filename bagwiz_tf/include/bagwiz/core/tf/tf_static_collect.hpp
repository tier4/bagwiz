// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__TF__TF_STATIC_COLLECT_HPP_
#define BAGWIZ__CORE__TF__TF_STATIC_COLLECT_HPP_

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <filesystem>
#include <string>
#include <vector>

// Reads a bag's static TF (topics whose name ends with "tf_static", type
// tf2_msgs/msg/TFMessage) into plain TransformStamped structs. Unlike
// tf_buffer_loader, which feeds a tf2::BufferCore and so keeps only the
// resolved tree, this hands back the raw transforms per topic — what commands
// that re-serialise or re-render the static TF need (`bagwiz tf static cp`,
// `bagwiz tf static dump`).
namespace bagwiz::core
{

// The transforms of one static TF topic. Deduplicated by child_frame_id (last
// value wins), so a topic carrying the same child twice collapses to one entry;
// first-seen order is preserved.
struct StaticTopicTransforms
{
  std::string name;
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
};

// How much of each static topic to read.
enum class StaticTfRead {
  // Every message on every static topic, collapsed per child_frame_id. Needed
  // when a later message may carry an edge the first one does not, or a newer
  // value for one it does.
  kWholeTopic,
  // Stop at each topic's first message, then stop reading the bag once every
  // static topic has produced one. Static TF is latched — a broadcaster sends
  // its whole set in one message and republishes that same set — so the first
  // message is normally the complete tree, and this skips the thousands of
  // identical republications a long recording accumulates. It does drop edges
  // that only a later message introduces, which happens when several
  // broadcasters publish disjoint subsets to one topic.
  kFirstMessagePerTopic,
};

// Read `bag_path`'s static TF topics and return their transforms per topic, in
// the order the topics appear in the bag. Topics that yield no transforms are
// dropped, so an empty result means the bag has no static TF worth acting on (a
// bag with a declared but empty /tf_static is indistinguishable from one with no
// /tf_static at all here — both are "nothing to do"). Throws std::runtime_error
// on open / decoder / decode failure.
std::vector<StaticTopicTransforms> collect_static_tf(
  const std::filesystem::path & bag_path, StaticTfRead mode);

}  // namespace bagwiz::core

#endif  // BAGWIZ__CORE__TF__TF_STATIC_COLLECT_HPP_
