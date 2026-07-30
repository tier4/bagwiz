// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__MAP_SLAM_COLORIZE_HPP_
#define COMMANDS__MAP_SLAM_COLORIZE_HPP_

#include "bagwiz/core/image/camera_info.hpp"
#include "bagwiz/core/slam/colorize_keyframe.hpp"
#include "bagwiz/core/slam/map_colorizer.hpp"
#include "bagwiz/core/slam/sensor_transform.hpp"
#include "bagwiz/core/tf/trajectory.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

// Construction internals of the `map slam --color` colorize pass, split out of
// map_slam.cpp so the thread-count rule and the per-camera colorizer setup
// can be unit-tested without driving a SLAM run. CLI-internal: this header
// lives with the command sources and is not installed.
namespace bagwiz::commands
{

// Effective thread count for the colorize pass: --threads resolved the same
// way as the mapping run (0 = hardware concurrency, positive values capped at
// it).
[[nodiscard]] int colorize_thread_count(int num_threads);

// --cam-info override entries ("<image_topic>=<info_topic>") parsed into a
// lookup keyed by image topic, or a human-readable error (empty on success).
// Image topics without an entry are absent from the map (their CameraInfo
// auto-resolves from the topic name).
struct CameraInfoOverrides
{
  std::unordered_map<std::string, std::string> by_image_topic;
  std::string error;
};

// Parse and validate the --cam-info entries against the listed camera image
// topics (`image_topics` is the --color ∪ --cam union: one --cam-info entry
// serves whichever role(s) named the topic). Errors: an entry without '=' (or
// with an empty half), an entry whose <image_topic> is not in `image_topics`,
// and a duplicate <image_topic>.
[[nodiscard]] CameraInfoOverrides parse_camera_info_overrides(
  std::span<const std::string> entries, std::span<const std::string> image_topics);

// The camera-independent geometry pre-pass (kd-tree, normals, spacings),
// built once and shared between every camera's MapColorizer (the kd-tree
// build is the expensive part). The neighbor count is the MapColorizerConfig
// default. `points` must outlive the returned geometry and must not be
// modified while it is in use.
[[nodiscard]] std::shared_ptr<const core::slam::ColorizeGeometry> build_shared_colorize_geometry(
  std::span<const std::array<float, 3>> points, int threads);

// One MapColorizer per camera, parallel to `camera_infos` / `t_cloud_cams`,
// all sharing the geometry pre-pass. `range_max` reuses the SLAM range crop:
// geometry farther than --max-range from any single viewpoint was never
// captured in one scan either. `points` / `trajectory` must outlive the
// returned colorizers.
//
// When `use_gpu` is true and this binary was built with BAGWIZ_WITH_SLAM_CUDA,
// a CUDA ColorizeRasterizer is injected into each MapColorizer; otherwise the
// CPU rasterizer is used.
[[nodiscard]] std::vector<std::unique_ptr<core::slam::MapColorizer>> build_camera_colorizers(
  std::span<const core::image::CameraInfo> camera_infos,
  std::span<const core::slam::SensorTransform> t_cloud_cams, double range_max, int threads,
  bool use_gpu, std::shared_ptr<const core::slam::ColorizeGeometry> geometry,
  std::span<const std::array<float, 3>> points, std::span<const core::TrajectoryPose> trajectory);

// Decode one camera image message (`type` + `payload`, exactly what the bag
// delivered) and hand the frame to `colorizer` — routed through
// `blur_picker`'s best-of-bucket path when non-null (--color-keyframe-blur; the
// picker must be blur-configured), else straight to add_image. The frame is
// stamped with the decoded header.stamp, falling back to `fallback_stamp_ns`
// (the bag record time) when the publisher left it unset. `dynamic_points` is
// the paired scan's occluder geometry (see ScanImagePairer); the blur path
// copies it into the buffered frame, the direct path passes the span through.
//
// This is THE per-frame colorize step, shared verbatim by the serial path and
// the per-camera worker threads so the two cannot diverge; it must be called
// from one thread per (colorizer, picker) pair, in frame arrival order.
// Returns false when the payload does not decode (the caller counts it);
// exceptions from the picker or the colorizer propagate to the caller.
[[nodiscard]] bool colorize_one_image(
  core::slam::MapColorizer & colorizer, core::slam::ColorizeKeyframePicker * blur_picker,
  std::string_view type, std::span<const std::byte> payload, std::int64_t fallback_stamp_ns,
  std::span<const std::array<float, 3>> dynamic_points);

// End of one camera's stream: dispatch `blur_picker`'s final gate bucket (its
// buffered sharpest frame, if any) to `colorizer`. No-op when `blur_picker`
// is null. Same threading rule as colorize_one_image.
void colorize_flush_keyframes(
  core::slam::MapColorizer & colorizer, core::slam::ColorizeKeyframePicker * blur_picker);

}  // namespace bagwiz::commands

#endif  // COMMANDS__MAP_SLAM_COLORIZE_HPP_
