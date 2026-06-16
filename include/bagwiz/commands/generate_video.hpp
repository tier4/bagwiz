// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__GENERATE_VIDEO_HPP_
#define BAGWIZ__COMMANDS__GENERATE_VIDEO_HPP_

#include "bagwiz/core/color/color_map.hpp"
#include "bagwiz/core/pointcloud/types.hpp"

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace bagwiz::commands
{

// Arguments for `bagwiz generate video`. Populated by GenerateCommand's CLI
// wiring (src/commands/generate.cpp) and consumed by run_generate_video. Kept
// in a header so the run function and the source check can be exercised
// directly from tests without driving the CLI parser.
struct GenerateVideoArgs
{
  std::filesystem::path input_path;
  std::string topic;
  std::filesystem::path output_path;
  // Replace a pre-existing <output>. Without it, an existing output path stops
  // the run before any work is done.
  bool overwrite = false;

  std::vector<std::string> pcd_topics{};
  std::optional<std::string> camera_info_topic{std::nullopt};
  core::pointcloud::ColorBy color_by = core::pointcloud::ColorBy::kDistance;
  core::color::ColorMapName color_map = core::color::ColorMapName::kJet;
  std::uint8_t pcd_point_size = 1;
};

// Classification of whether a topic can be rendered to video.
enum class VideoSourceStatus {
  kOk,                     // topic present and a supported message type
  kInputUnopenable,        // the bag could not be opened
  kTopicNotFound,          // no topic by that name in the bag
  kUnsupportedType,        // topic present but its message type is not renderable
  kPcdTopicInvalid,        // a --pcd topic is missing or not sensor_msgs/msg/PointCloud2
  kCameraInfoTopicInvalid  // the CameraInfo topic is missing or not sensor_msgs/msg/CameraInfo
};

// Outcome of check_video_source(). `topic_type` is set whenever the topic was
// found (kOk or kUnsupportedType); `camera_info_topic` is set when the check
// resolved a CameraInfo topic for pointcloud overlay; `message` is a
// human-readable, log-verbatim explanation on any non-kOk status. Never throws.
struct VideoSourceCheck
{
  VideoSourceStatus status = VideoSourceStatus::kInputUnopenable;
  std::string topic_type;
  std::string message;
  std::optional<std::string> camera_info_topic;

  [[nodiscard]] bool ok() const noexcept { return status == VideoSourceStatus::kOk; }
};

// Resolve `topic` against the bag at `input` and classify whether it can be
// rendered to video: the message type must be sensor_msgs/msg/Image or
// sensor_msgs/msg/CompressedImage. Reads only the topic list, never payloads.
[[nodiscard]] VideoSourceCheck check_video_source(
  const std::filesystem::path & input, const std::string & topic);

// Extended source check used when pointcloud overlay is requested. Validates
// the image topic, each --pcd topic, and resolves the CameraInfo topic (from
// --camera-info if provided, otherwise inferred from the image topic). The
// resolved CameraInfo topic is returned in `camera_info_topic` on success.
[[nodiscard]] VideoSourceCheck check_video_source(
  const std::filesystem::path & input, const GenerateVideoArgs & args);

// Render `args.topic` from `args.input_path` to a video at `args.output_path`,
// inferring the container/codec from the output extension and the frame rate
// from the message timestamps. Returns a process exit code: 0 on success, 1 on
// any error. Renders raw sensor_msgs/msg/Image (bgr8 / rgb8) and
// sensor_msgs/msg/CompressedImage (JPEG / PNG, decoded to BGR before encoding).
int run_generate_video(const GenerateVideoArgs & args);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__GENERATE_VIDEO_HPP_
