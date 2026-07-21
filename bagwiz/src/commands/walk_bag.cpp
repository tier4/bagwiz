// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "walk_bag.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/core/image/camera_info_resolver.hpp"
#include "bagwiz/io/bag_open.hpp"
#include "bagwiz/io/topics.hpp"

#include <string>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{

constexpr const char * kPointCloudType = "sensor_msgs/msg/PointCloud2";

}  // namespace

std::optional<OpenedBag> open_bag_and_find_topic(
  const std::filesystem::path & path, const std::string & topic, const char * logger)
{
  auto reader = io::open_read_or_log(path, logger);
  if (!reader) {
    return std::nullopt;
  }
  const io::TopicInfo * topic_info = io::find_topic_or_log(*reader, topic, path, logger);
  if (topic_info == nullptr) {
    return std::nullopt;
  }
  return OpenedBag{std::move(reader), topic_info};
}

std::vector<std::string> collect_pcd_topics(const io::BagReader & reader)
{
  std::vector<std::string> pcd_topics;
  for (const auto & t : reader.topics()) {
    if (t.type == kPointCloudType) {
      pcd_topics.push_back(t.name);
    }
  }
  return pcd_topics;
}

WalkCameraInfo resolve_walk_camera_info(
  const std::filesystem::path & input, const std::string & topic,
  const std::optional<std::string> & explicit_topic, std::span<const io::TopicInfo> topics)
{
  WalkCameraInfo result;
  std::optional<std::string> camera_info_topic;

  if (explicit_topic.has_value()) {
    if (const auto err = core::camera_info::validate_camera_info_topic(input, *explicit_topic);
        err.has_value()) {
      result.error = *err;
    } else {
      camera_info_topic = *explicit_topic;
    }
  } else {
    const auto resolution = core::camera_info::resolve_camera_info_topic(topic, topics);
    camera_info_topic = resolution.topic;
    if (resolution.error.has_value()) {
      result.error = *resolution.error;
    }
  }

  if (camera_info_topic.has_value()) {
    const auto ci = core::camera_info::load_camera_info(input, *camera_info_topic);
    if (ci.ok()) {
      result.info = *ci.info;
    } else {
      result.error = ci.error;
    }
  }
  return result;
}

}  // namespace bagwiz::commands
