// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__GENERATE_VIDEO_OVERLAY_HPP_
#define BAGWIZ__COMMANDS__GENERATE_VIDEO_OVERLAY_HPP_

#include "bagwiz/core/camera/camera_info.hpp"
#include "bagwiz/core/color/color_map.hpp"
#include "bagwiz/core/pointcloud/point_cloud_reader.hpp"
#include "bagwiz/core/pointcloud/types.hpp"
#include "bagwiz/core/video/video_encoder.hpp"

#include <tf2/buffer_core.hpp>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace bagwiz::commands
{

// Cached per-topic pointcloud state for `generate video` overlay. Holds the
// latest payload and its decoded view; extraction is repeated only when the
// payload changes (a new message arrived for this topic).
class PcdOverlayState
{
public:
  // Replace the cached payload with `payload` and re-extract the view. Returns
  // an empty string on success or an error message if the payload could not be
  // decoded as sensor_msgs/msg/PointCloud2.
  [[nodiscard]] std::string update(std::span<const std::byte> payload);

  [[nodiscard]] bool has_data() const noexcept { return view_.has_value(); }
  [[nodiscard]] const core::pointcloud::PointCloudView & view() const { return *view_; }

private:
  std::vector<std::byte> payload_;
  std::optional<core::pointcloud::PointCloudView> view_;
};

// Paint pointcloud overlays onto `mutable_pixels`. `fw`/`fh` are the image
// dimensions, `fstep` is the row stride in bytes, and `fsrc` selects the
// channel order. `image_timestamp_ns` is converted to a tf2 time point for the
// TF lookup. Returns an empty string on success or a fatal error message;
// per-topic TF lookup failures are logged as warnings and skipped.
[[nodiscard]] std::string paint_pointcloud_overlays(
  std::span<std::byte> mutable_pixels, std::uint32_t fw, std::uint32_t fh, std::uint32_t fstep,
  core::video::SourcePixelFormat fsrc, std::uint64_t frame_index,
  const std::vector<std::string> & pcd_topics, std::vector<PcdOverlayState> & pcd_states,
  const core::camera::CameraInfo & camera_info, const tf2::BufferCore & tf_buffer,
  std::int64_t image_timestamp_ns, core::pointcloud::ColorBy color_by,
  core::color::ColorMapName color_map);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__GENERATE_VIDEO_OVERLAY_HPP_
