// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tf_static_loader.hpp"

#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/tf_value_extract.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <tf2/buffer_core.hpp>
#include <tf2/time.hpp>

#include <chrono>
#include <memory>
#include <stdexcept>
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

// Static transforms are time-independent, but tf2::BufferCore still needs a
// cache duration. One year dwarfs any realistic bag and matches the buffer
// sizing used elsewhere in bagwiz.
constexpr std::chrono::hours kTfStaticCacheTime{24 * 365};

bool is_static_tf_topic(std::string_view topic_name)
{
  if (topic_name.size() < kTfStaticSuffix.size()) {
    return false;
  }
  return topic_name.compare(
           topic_name.size() - kTfStaticSuffix.size(), kTfStaticSuffix.size(), kTfStaticSuffix) ==
         0;
}

}  // namespace

TfStaticLoadResult load_static_tf(const std::filesystem::path & input)
{
  TfStaticLoadResult result;
  result.buffer = std::make_unique<tf2::BufferCore>(kTfStaticCacheTime);

  try {
    auto reader = io::open_read(input);
    reader->populate_schemas();

    std::vector<std::string> static_tf_topics;
    for (const auto & topic : reader->topics()) {
      if (topic.type == kTfMessageType && is_static_tf_topic(topic.name)) {
        static_tf_topics.push_back(topic.name);
      }
    }

    if (static_tf_topics.empty()) {
      result.ok = true;
      return result;
    }

    io::ReadFilter filter;
    filter.topics = static_tf_topics;
    reader->set_filter(filter);

    std::unordered_map<std::string, std::unique_ptr<decoder::Decoder>> decoder_by_topic;
    for (const auto & topic_info : reader->topics()) {
      if (topic_info.type != kTfMessageType) {
        continue;
      }
      if (!is_static_tf_topic(topic_info.name)) {
        continue;
      }
      auto open = decoder::open_decoder(topic_info);
      if (!open.ok()) {
        throw std::runtime_error(
          "Could not open decoder for static TF topic '" + topic_info.name + "': " + open.error);
      }
      decoder_by_topic.emplace(topic_info.name, std::move(open.decoder));
    }

    io::RawMessage raw;
    while (reader->next(raw)) {
      auto it = decoder_by_topic.find(raw.topic->name);
      if (it == decoder_by_topic.end()) {
        continue;
      }
      const auto decoded = it->second->decode(raw.payload);
      if (!decoded.ok()) {
        throw std::runtime_error(
          "Failed to decode static TF message on '" + raw.topic->name + "': " + decoded.error);
      }
      for (const auto & transform : extract_tf_message(*decoded.value)) {
        result.buffer->setTransform(transform, "bagwiz", /*is_static=*/true);
      }
    }

    result.ok = true;
  } catch (const std::exception & e) {
    result.ok = false;
    result.error = e.what();
    result.buffer.reset();
  }
  return result;
}

}  // namespace bagwiz::core
