// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/generate_video_overlay.hpp"

#include "bagwiz/core/logging.hpp"
#include "bagwiz/core/pointcloud/project.hpp"

#include <tf2/time.hpp>

#include <tf2/exceptions.h>

#include <chrono>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace bagwiz::commands
{

namespace
{
constexpr const char * kLogger = "bagwiz.cmd.generate";

void paint_projected_points(
  std::span<std::byte> mutable_pixels, std::uint32_t fw, std::uint32_t fh, std::uint32_t fstep,
  core::video::SourcePixelFormat fsrc, const std::vector<core::pointcloud::ProjectedPoint> & points,
  std::uint8_t point_size)
{
  const std::int32_t radius = static_cast<std::int32_t>(point_size) / 2;
  for (const auto & p : points) {
    for (std::int32_t dy = -radius; dy <= radius; ++dy) {
      const auto vy = static_cast<std::int32_t>(p.v) + dy;
      if (vy < 0 || vy >= static_cast<std::int32_t>(fh)) {
        continue;
      }
      for (std::int32_t dx = -radius; dx <= radius; ++dx) {
        const auto vx = static_cast<std::int32_t>(p.u) + dx;
        if (vx < 0 || vx >= static_cast<std::int32_t>(fw)) {
          continue;
        }
        const std::size_t idx =
          static_cast<std::size_t>(vy) * fstep + static_cast<std::size_t>(vx) * 3U;
        if (idx + 2 >= mutable_pixels.size()) {
          continue;
        }
        if (fsrc == core::video::SourcePixelFormat::kBgr8) {
          mutable_pixels[idx] = static_cast<std::byte>(p.rgb.b);
          mutable_pixels[idx + 1] = static_cast<std::byte>(p.rgb.g);
          mutable_pixels[idx + 2] = static_cast<std::byte>(p.rgb.r);
        } else {
          mutable_pixels[idx] = static_cast<std::byte>(p.rgb.r);
          mutable_pixels[idx + 1] = static_cast<std::byte>(p.rgb.g);
          mutable_pixels[idx + 2] = static_cast<std::byte>(p.rgb.b);
        }
      }
    }
  }
}

}  // namespace

std::string PcdOverlayState::update(std::span<const std::byte> payload)
{
  payload_.assign(payload.begin(), payload.end());
  const auto result = core::pointcloud::extract_point_cloud(payload_);
  if (!result.ok()) {
    view_.reset();
    return result.error;
  }
  view_ = *result.view;
  return {};
}

std::string paint_pointcloud_overlays(
  std::span<std::byte> mutable_pixels, const std::uint32_t fw, const std::uint32_t fh,
  const std::uint32_t fstep, const core::video::SourcePixelFormat fsrc,
  const std::uint64_t frame_index, const std::vector<std::string> & pcd_topics,
  std::vector<PcdOverlayState> & pcd_states, const core::camera::CameraInfo & camera_info,
  const tf2::BufferCore & tf_buffer, const std::int64_t image_timestamp_ns,
  const core::pointcloud::ColorBy color_by, const core::color::ColorMapName color_map,
  const std::uint8_t point_size)
{
  const auto time_point = tf2::TimePoint(std::chrono::nanoseconds(image_timestamp_ns));

  for (std::size_t i = 0; i < pcd_topics.size(); ++i) {
    if (!pcd_states[i].has_data()) {
      continue;
    }

    const auto & cloud_view = pcd_states[i].view();

    tf2::Transform cloud_to_camera;
    try {
      const auto stamped =
        tf_buffer.lookupTransform(camera_info.frame_id, cloud_view.frame_id, time_point);
      const auto & t = stamped.transform;
      cloud_to_camera.setOrigin(tf2::Vector3(t.translation.x, t.translation.y, t.translation.z));
      cloud_to_camera.setRotation(
        tf2::Quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w));
    } catch (const tf2::TransformException & e) {
      BAGWIZ_LOG_WARN(
        kLogger, "frame %" PRIu64 ", topic '%s': TF lookup failed: %s", frame_index,
        pcd_topics[i].c_str(), e.what());
      continue;
    }

    const auto projection = core::pointcloud::project_point_cloud(
      cloud_view, camera_info, cloud_to_camera, color_by, color_map);
    if (!projection.ok()) {
      return "topic '" + pcd_topics[i] + "': projection failed: " + projection.error;
    }

    // Points are already depth-sorted front-to-back by project_point_cloud.
    paint_projected_points(mutable_pixels, fw, fh, fstep, fsrc, *projection.points, point_size);
  }

  return {};
}

}  // namespace bagwiz::commands
