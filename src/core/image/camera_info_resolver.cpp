// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/image/camera_info_resolver.hpp"

#include <memory>
#include <string>
#include <string_view>

namespace bagwiz::core::camera_info
{
namespace
{
constexpr std::string_view kCameraInfoType = "sensor_msgs/msg/CameraInfo";
constexpr std::string_view kCameraInfoSuffix = "/camera_info";

bool strip_image_suffix(std::string_view & stem, std::string_view suffix)
{
  if (stem.size() > suffix.size() && stem.ends_with(suffix)) {
    stem.remove_suffix(suffix.size());
    return true;
  }
  return false;
}
}  // namespace

std::optional<std::string> resolve_camera_info_topic_name(const std::string & image_topic)
{
  std::string_view stem{image_topic};
  if (
    strip_image_suffix(stem, "/image_rect_color/compressed") ||
    strip_image_suffix(stem, "/image_raw/compressed") ||
    strip_image_suffix(stem, "/image_rect_color") || strip_image_suffix(stem, "/image_raw")) {
    return std::string{stem} + std::string{kCameraInfoSuffix};
  }
  return std::nullopt;
}

CameraInfoResolution resolve_camera_info_topic(
  const std::string & image_topic, std::span<const io::TopicInfo> topics)
{
  CameraInfoResolution result;
  const auto candidate = resolve_camera_info_topic_name(image_topic);
  if (!candidate) {
    result.error = "cannot derive camera_info topic from '" + image_topic + "'";
    return result;
  }
  for (const auto & t : topics) {
    if (t.name == *candidate) {
      if (t.type == kCameraInfoType) {
        result.topic = *candidate;
        return result;
      }
      result.error = "topic '" + *candidate + "' has type '" + t.type +
                     "', but camera_info requires " + std::string{kCameraInfoType};
      return result;
    }
  }
  result.error = "camera_info topic '" + *candidate + "' not found";
  return result;
}

std::optional<std::string> validate_camera_info_topic(
  const std::filesystem::path & input, const std::string & topic)
{
  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(input);
  } catch (const std::exception & e) {
    return "failed to open '" + input.string() + "': " + e.what();
  }
  for (const auto & t : reader->topics()) {
    if (t.name == topic) {
      if (t.type != kCameraInfoType) {
        return "topic '" + topic + "' has type '" + t.type + "', but camera_info requires " +
               std::string{kCameraInfoType};
      }
      return std::nullopt;
    }
  }
  return "camera_info topic '" + topic + "' not found in " + input.string();
}

core::image::CameraInfoResult load_camera_info(
  const std::filesystem::path & input, const std::string & topic)
{
  std::unique_ptr<io::BagReader> reader;
  try {
    reader = io::open_read(input);
  } catch (const std::exception & e) {
    core::image::CameraInfoResult result;
    result.error = "failed to open '" + input.string() + "': " + e.what();
    return result;
  }
  io::ReadFilter filter;
  filter.topics.push_back(topic);
  try {
    reader->set_filter(filter);
    io::RawMessage raw;
    while (reader->next(raw)) {
      if (raw.topic && raw.topic->name == topic) {
        return core::image::extract_camera_info(raw.payload);
      }
    }
  } catch (const std::exception & e) {
    core::image::CameraInfoResult result;
    result.error = "error reading camera-info topic '" + topic + "': " + e.what();
    return result;
  }
  core::image::CameraInfoResult result;
  result.error = "no messages found on camera-info topic '" + topic + "'";
  return result;
}

}  // namespace bagwiz::core::camera_info
