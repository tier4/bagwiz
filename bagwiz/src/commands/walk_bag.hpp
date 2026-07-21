// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__WALK_BAG_HPP_
#define COMMANDS__WALK_BAG_HPP_

#include "bagwiz/core/image/camera_info.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

// Bag-side orchestration of `bagwiz walk`: opening the bag and locating the
// walked topic, listing the overlay-eligible PointCloud2 topics, and
// resolving the CameraInfo used by the undistort/projection preview. Thin
// wrappers over the Phase-3 shared helpers, split out of walk.cpp.
// CLI-internal: this header lives with the command sources and is not
// installed.
namespace bagwiz::commands
{

// Result of open_bag_and_find_topic(). `topic_info` aliases the reader's
// internal topic list, so the reader must stay alive while it is used.
struct OpenedBag
{
  std::unique_ptr<io::BagReader> reader;
  const io::TopicInfo * topic_info = nullptr;
};

// Open `path` for reading and locate `topic`, logging the open-or-log
// helpers' fixed messages on failure. Returns std::nullopt on either
// failure; the caller's error path stays `if (!opened) return 1;`.
[[nodiscard]] std::optional<OpenedBag> open_bag_and_find_topic(
  const std::filesystem::path & path, const std::string & topic, const char * logger);

// List the bag's PointCloud2 topics (the overlay picker's candidates). This
// deliberately does NOT count messages to filter out empty topics: on a
// sqlite3 bag without a topic_id index the count requires a full messages
// table scan, which stalls walk's startup on multi-GB bags for a topic the
// user may never open. An empty topic picked in the overlay fails at
// initialization with a "has no messages" status instead.
[[nodiscard]] std::vector<std::string> collect_pcd_topics(const io::BagReader & reader);

// Outcome of resolve_walk_camera_info(). `error` is empty on success; it
// carries the resolution/validation/loading reason otherwise (walk surfaces
// it when the user toggles undistort or the pcd projection).
struct WalkCameraInfo
{
  std::optional<core::image::CameraInfo> info;
  std::string error;
};

// Resolve the CameraInfo for `topic`: validate `explicit_topic` when given
// (--cam-info), otherwise derive the candidate from the image topic name,
// then load the first CameraInfo message from the bag.
[[nodiscard]] WalkCameraInfo resolve_walk_camera_info(
  const std::filesystem::path & input, const std::string & topic,
  const std::optional<std::string> & explicit_topic, std::span<const io::TopicInfo> topics);

}  // namespace bagwiz::commands

#endif  // COMMANDS__WALK_BAG_HPP_
