// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__IMAGE__CAMERA_INFO_RESOLVER_HPP_
#define BAGWIZ__CORE__IMAGE__CAMERA_INFO_RESOLVER_HPP_

#include "bagwiz/core/image/camera_info.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <filesystem>
#include <optional>
#include <span>
#include <string>

namespace bagwiz::core::camera_info
{

// Resolve the CameraInfo topic name from an image topic name using the standard
// suffix rules. Returns std::nullopt when the image topic name does not match
// any known pattern.
[[nodiscard]] std::optional<std::string> resolve_camera_info_topic_name(
  const std::string & image_topic);

struct CameraInfoResolution
{
  std::optional<std::string> topic;
  std::optional<std::string> error;
};

// Resolve and validate the candidate against a bag's topic list. The candidate
// must exist and have type sensor_msgs/msg/CameraInfo.
[[nodiscard]] CameraInfoResolution resolve_camera_info_topic(
  const std::string & image_topic, std::span<const io::TopicInfo> topics);

// Validate an explicit camera-info topic: it must exist in the bag and have the
// expected type. Returns a human-readable error on failure, or std::nullopt on
// success.
[[nodiscard]] std::optional<std::string> validate_camera_info_topic(
  const std::filesystem::path & input, const std::string & topic);

// Read the first CameraInfo message for `topic` from `input`.
[[nodiscard]] core::image::CameraInfoResult load_camera_info(
  const std::filesystem::path & input, const std::string & topic);

}  // namespace bagwiz::core::camera_info

#endif  // BAGWIZ__CORE__IMAGE__CAMERA_INFO_RESOLVER_HPP_
