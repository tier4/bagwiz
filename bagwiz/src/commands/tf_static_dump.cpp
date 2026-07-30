// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/tf_static_dump.hpp"

#include "bagwiz/core/base/atomic_write.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/base/output_path.hpp"
#include "bagwiz/core/tf/tf_static_collect.hpp"
#include "bagwiz/core/tf/tf_static_tree_yaml.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <fmt/core.h>

#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <unordered_map>
#include <vector>

namespace bagwiz::commands
{

namespace
{

constexpr const char * kLogger = "bagwiz.cmd.tf.static.dump";

// Merge every static topic's latched transforms into the one tree the YAML
// schema can hold, in first-seen order. Each topic's own duplicates are already
// collapsed by core::collect_static_tf, so the only case left is the same
// child_frame_id arriving from two topics: identical parents are fine (last
// wins, matching the within-topic rule), different parents contradict each
// other and cannot both be written under a single child key. Returns false
// (after logging) on such a contradiction.
bool merge_static_topics(
  const std::vector<core::StaticTopicTransforms> & topics,
  std::vector<geometry_msgs::msg::TransformStamped> & out)
{
  std::unordered_map<std::string, std::size_t> index_by_child;
  // Which topic each merged transform came from, so a conflict can name both
  // sides rather than just the frame.
  std::vector<std::string> source_topic;
  for (const auto & topic : topics) {
    for (const auto & t : topic.transforms) {
      const auto ins = index_by_child.emplace(t.child_frame_id, out.size());
      if (ins.second) {
        out.push_back(t);
        source_topic.push_back(topic.name);
        continue;
      }
      const std::size_t existing = ins.first->second;
      if (out[existing].header.frame_id != t.header.frame_id) {
        BAGWIZ_LOG_ERROR(
          kLogger,
          "Static TF topics disagree on frame '%s': '%s' gives it parent '%s' but '%s' gives it "
          "parent '%s'. One tree cannot hold both, so nothing was dumped.",
          t.child_frame_id.c_str(), source_topic[existing].c_str(),
          out[existing].header.frame_id.c_str(), topic.name.c_str(), t.header.frame_id.c_str());
        return false;
      }
      out[existing] = t;
      source_topic[existing] = topic.name;
    }
  }
  return true;
}

}  // namespace

int run_tf_static_dump(
  const std::filesystem::path & input_path,
  const std::optional<std::filesystem::path> & output_path, bool overwrite)
{
  std::vector<core::StaticTopicTransforms> static_topics;
  try {
    // First message only: static TF is latched, so a broadcaster's first message
    // already carries its whole set and every republication after it repeats
    // that set. Reading one message per topic is enough for the tree and stops
    // the read early instead of streaming a long recording's thousands of
    // identical republications.
    static_topics = core::collect_static_tf(input_path, core::StaticTfRead::kFirstMessagePerTopic);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Failed to read static TF from %s: %s", input_path.c_str(), e.what());
    return 1;
  }
  if (static_topics.empty()) {
    BAGWIZ_LOG_ERROR(
      kLogger,
      "Bag %s has no static tf2_msgs/msg/TFMessage topic (e.g. /tf_static) carrying transforms; "
      "nothing to dump.",
      input_path.c_str());
    return 1;
  }

  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  if (!merge_static_topics(static_topics, transforms)) {
    return 1;
  }

  const std::string yaml = core::emit_static_tf_tree_yaml(
    std::span<const geometry_msgs::msg::TransformStamped>(transforms.data(), transforms.size()),
    input_path.string());
  const auto transform_count = static_cast<std::uint64_t>(transforms.size());
  const auto topic_count = static_cast<std::uint64_t>(static_topics.size());

  if (!output_path.has_value()) {
    // The tree is this command's data output, so it goes to stdout while every
    // diagnostic above went to stderr -- `bagwiz tf static dump -i <bag> >
    // tf_static.yaml` is pipe-clean. See core/base/logging.hpp.
    fmt::print(stdout, "{}", yaml);
    if (std::fflush(stdout) != 0) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to write the static TF tree to stdout");
      return 1;
    }
    BAGWIZ_LOG_INFO(
      kLogger,
      "tf static dump: wrote %" PRIu64 " transform(s) from %" PRIu64
      " static topic(s) in '%s' to stdout.",
      transform_count, topic_count, input_path.string().c_str());
    return 0;
  }

  // Claim the output only now, once the run is certain to produce a tree:
  // prepare_output_path() deletes an existing path under --overwrite, so doing
  // this any earlier would let a bag with no static TF (or a contradiction
  // between its topics) destroy the user's -o file and then fail. Only the read
  // can tell those cases apart, so every refusal above has to come first.
  if (const auto r = core::prepare_output_path(*output_path, overwrite); !r.ok) {
    BAGWIZ_LOG_ERROR(kLogger, "%s", r.error.c_str());
    return 1;
  }

  std::string error;
  if (!core::write_file_atomically(*output_path, yaml, error)) {
    BAGWIZ_LOG_ERROR(kLogger, "Could not write '%s': %s", output_path->c_str(), error.c_str());
    return 1;
  }

  BAGWIZ_LOG_INFO(
    kLogger,
    "tf static dump: wrote '%s' with %" PRIu64 " transform(s) from %" PRIu64
    " static topic(s) in '%s'.",
    output_path->string().c_str(), transform_count, topic_count, input_path.string().c_str());
  return 0;
}

}  // namespace bagwiz::commands
