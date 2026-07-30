// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tf/tf_static_collect.hpp"

#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/tf/tf_topics.hpp"
#include "bagwiz/core/tf/tf_value_extract.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace bagwiz::core
{

namespace
{

constexpr const char * kTfMessageType = "tf2_msgs/msg/TFMessage";

// Open one decoder per static TF topic in `reader`. Throws std::runtime_error
// if a decoder cannot be constructed (no embedded schema and no typesupport).
std::unordered_map<std::string, std::unique_ptr<decoder::Decoder>> open_static_tf_decoders(
  const io::BagReader & reader)
{
  std::unordered_map<std::string, std::unique_ptr<decoder::Decoder>> decoder_by_topic;
  for (const auto & topic_info : reader.topics()) {
    if (topic_info.type != kTfMessageType || !is_static_tf_topic(topic_info.name)) {
      continue;
    }
    auto open = decoder::open_decoder(topic_info);
    if (!open.ok()) {
      throw std::runtime_error(
        "Could not open decoder for static TF topic '" + topic_info.name + "': " + open.error);
    }
    decoder_by_topic.emplace(topic_info.name, std::move(open.decoder));
  }
  return decoder_by_topic;
}

// Merge one message's transforms into a topic's accumulator: a child_frame_id
// already seen is overwritten in place (last wins) so a re-published static
// topic collapses to its latched set; a new one is appended (first-seen order).
void merge_transforms(
  std::vector<geometry_msgs::msg::TransformStamped> & transforms,
  std::unordered_map<std::string, std::size_t> & child_index,
  const std::vector<geometry_msgs::msg::TransformStamped> & incoming)
{
  for (const auto & t : incoming) {
    const auto ins = child_index.emplace(t.child_frame_id, transforms.size());
    if (ins.second) {
      transforms.push_back(t);
    } else {
      transforms[ins.first->second] = t;
    }
  }
}

}  // namespace

std::vector<StaticTopicTransforms> collect_static_tf(
  const std::filesystem::path & bag_path, StaticTfRead mode)
{
  auto reader = io::open_read(bag_path);
  reader->populate_schemas();

  std::vector<std::string> static_topics;
  for (const auto & t : reader->topics()) {
    if (t.type == kTfMessageType && is_static_tf_topic(t.name)) {
      static_topics.push_back(t.name);
    }
  }
  if (static_topics.empty()) {
    return {};
  }

  auto decoder_by_topic = open_static_tf_decoders(*reader);

  io::ReadFilter filter;
  filter.topics = static_topics;
  reader->set_filter(filter);

  // Per topic: the accumulating transform list plus a child_frame_id -> index
  // map (see merge_transforms) keyed by topic name.
  std::unordered_map<std::string, std::vector<geometry_msgs::msg::TransformStamped>> by_topic;
  std::unordered_map<std::string, std::unordered_map<std::string, std::size_t>>
    child_index_by_topic;

  // kFirstMessagePerTopic: the topics still waiting for their first message. A
  // topic leaves the set once it has one, and an empty set ends the read — no
  // point streaming the rest of a multi-gigabyte bag for messages that will be
  // ignored. The storage-level topic filter above cannot express "first only".
  std::unordered_set<std::string> pending;
  if (mode == StaticTfRead::kFirstMessagePerTopic) {
    pending.insert(static_topics.begin(), static_topics.end());
  }

  io::RawMessage raw;
  while (reader->next(raw)) {
    auto it = decoder_by_topic.find(raw.topic->name);
    if (it == decoder_by_topic.end()) {
      continue;
    }
    if (mode == StaticTfRead::kFirstMessagePerTopic && !pending.contains(raw.topic->name)) {
      continue;
    }
    const auto decoded = it->second->decode(raw.payload);
    if (!decoded.ok()) {
      throw std::runtime_error(
        "Failed to decode static TF message on '" + raw.topic->name + "': " + decoded.error);
    }
    merge_transforms(
      by_topic[raw.topic->name], child_index_by_topic[raw.topic->name],
      extract_tf_message(*decoded.value));
    if (mode == StaticTfRead::kFirstMessagePerTopic) {
      pending.erase(raw.topic->name);
      if (pending.empty()) {
        break;
      }
    }
  }

  std::vector<StaticTopicTransforms> out;
  for (const auto & name : static_topics) {
    auto found = by_topic.find(name);
    if (found == by_topic.end() || found->second.empty()) {
      continue;
    }
    out.push_back({name, std::move(found->second)});
  }
  return out;
}

}  // namespace bagwiz::core
