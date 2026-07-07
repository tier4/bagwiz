// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tf_buffer_loader.hpp"

#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/tf_value_extract.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace bagwiz::core
{

namespace
{

constexpr const char * kTfMessageType = "tf2_msgs/msg/TFMessage";
constexpr std::string_view kTfStaticSuffix = "tf_static";

bool is_static_tf_topic(std::string_view name)
{
  return name.size() >= kTfStaticSuffix.size() &&
         name.compare(
           name.size() - kTfStaticSuffix.size(), kTfStaticSuffix.size(), kTfStaticSuffix) == 0;
}

}  // namespace

std::optional<std::string> load_tf_buffer(
  const std::filesystem::path & input, tf2::BufferCore & buffer)
{
  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(input);
  } catch (const std::exception & e) {
    return "failed to open '" + input.string() + "': " + e.what();
  }

  std::vector<const io::TopicInfo *> tf_topics;
  for (const auto & t : reader->topics()) {
    if (t.type == kTfMessageType) {
      tf_topics.push_back(&t);
    }
  }
  if (tf_topics.empty()) {
    return "no tf2_msgs/msg/TFMessage topics found; cannot resolve point-cloud transform";
  }

  io::ReadFilter filter;
  for (const auto * t : tf_topics) {
    filter.topics.push_back(t->name);
  }
  reader->set_filter(filter);

  std::unordered_map<std::string, std::unique_ptr<decoder::Decoder>> decoders;
  for (const auto * t : tf_topics) {
    auto open = decoder::open_decoder(*t);
    if (!open.ok()) {
      return "could not open decoder for TF topic '" + t->name + "': " + open.error;
    }
    decoders.emplace(t->name, std::move(open.decoder));
  }

  io::RawMessage raw;
  try {
    while (reader->next(raw)) {
      auto it = decoders.find(raw.topic->name);
      if (it == decoders.end()) {
        continue;
      }
      const auto decoded = it->second->decode(raw.payload);
      if (!decoded.ok()) {
        return "failed to decode TF message on '" + raw.topic->name + "': " + decoded.error;
      }
      const auto transforms = extract_tf_message(*decoded.value);
      const bool is_static = is_static_tf_topic(raw.topic->name);
      for (const auto & t : transforms) {
        buffer.setTransform(t, "bagwiz", is_static);
      }
    }
  } catch (const std::exception & e) {
    return "error reading TF topics: " + std::string(e.what());
  }
  return std::nullopt;
}

std::optional<std::string> load_static_tf_buffer(
  const std::filesystem::path & input, tf2::BufferCore & buffer)
{
  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(input);
  } catch (const std::exception & e) {
    return std::string("failed to reopen bag for static TF: ") + e.what();
  }

  std::vector<const io::TopicInfo *> static_topics;
  for (const auto & t : reader->topics()) {
    if (t.type == kTfMessageType && is_static_tf_topic(t.name)) {
      static_topics.push_back(&t);
    }
  }
  if (static_topics.empty()) {
    // Caller-neutral: this helper is shared across commands with different
    // flags (pcd concat's --frame, pcd undistort's --from/--to, ...), so it
    // must not bake any one of them into the message. Callers that want
    // flag-specific context should prepend their own.
    return "bag has no static TF topic (…tf_static)";
  }

  io::ReadFilter filter;
  for (const auto * t : static_topics) {
    filter.topics.push_back(t->name);
  }
  reader->set_filter(filter);

  std::unordered_map<std::string, std::unique_ptr<decoder::Decoder>> decoders;
  for (const auto * t : static_topics) {
    auto open = decoder::open_decoder(*t);
    if (!open.ok()) {
      return "could not open decoder for '" + t->name + "': " + open.error;
    }
    decoders.emplace(t->name, std::move(open.decoder));
  }

  io::RawMessage raw;
  try {
    while (reader->next(raw)) {
      const auto it = decoders.find(raw.topic->name);
      if (it == decoders.end()) {
        continue;
      }
      const auto decoded = it->second->decode(raw.payload);
      if (!decoded.ok()) {
        return "failed to decode static TF on '" + raw.topic->name + "': " + decoded.error;
      }
      for (const auto & t : extract_tf_message(*decoded.value)) {
        buffer.setTransform(t, "bagwiz", /*is_static=*/true);
      }
    }
  } catch (const std::exception & e) {
    return std::string("error reading static TF: ") + e.what();
  }
  return std::nullopt;
}

}  // namespace bagwiz::core
