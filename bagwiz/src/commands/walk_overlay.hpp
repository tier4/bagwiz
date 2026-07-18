// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__WALK_OVERLAY_HPP_
#define COMMANDS__WALK_OVERLAY_HPP_

#include "bagwiz/core/image/camera_info.hpp"
#include "bagwiz/core/image/packed_raster.hpp"
#include "bagwiz/core/image/undistort.hpp"
#include "bagwiz/core/pointcloud/color_scheme.hpp"
#include "bagwiz/core/pointcloud/fetcher.hpp"
#include "bagwiz/core/pointcloud/property.hpp"
#include "bagwiz/core/tui/image/terminal_image_caps.hpp"
#include "bagwiz/core/tui/pager.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <tf2/buffer_core.hpp>

#include <cstdint>
#include <filesystem>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

// Point-cloud projection overlay of `bagwiz walk`'s image preview: the
// overlay state, the interactive topic picker, the one-shot initialization
// (TF buffer + per-topic scans), and the per-frame projection. Moved out of
// walk.cpp verbatim; the interactive parts stay TTY-coupled by design.
// CLI-internal: this header lives with the command sources and is not
// installed.
namespace bagwiz::commands
{

// Overlay key-handling state. The default view is distance coloured with the
// jet scheme, 2px points at full opacity over an auto-computed range.
struct PcdOverlayState
{
  bool enabled = false;
  std::vector<std::string> topics;
  core::pointcloud::PointCloudProperty property = core::pointcloud::PointCloudProperty::kDistance;
  core::pointcloud::ColorScheme scheme = core::pointcloud::ColorScheme::kJet;
  std::uint32_t point_size = 2;
  float alpha = 1.0f;
  bool auto_range = true;
  double manual_min = 0.0;
  double manual_max = 1.0;
  double computed_min = 0.0;
  double computed_max = 1.0;
  bool has_intensity = false;
  // Min/max of every colour property, captured once when the topics are
  // selected. Switching the active property then reuses these instead of
  // re-scanning the bag, so [f] is instant even with auto range on.
  core::pointcloud::PropertyRanges ranges;
};

// Display names used by the preview info row.
[[nodiscard]] std::string_view pcd_property_name(core::pointcloud::PointCloudProperty prop);
[[nodiscard]] std::string_view pcd_scheme_name(core::pointcloud::ColorScheme scheme);

class PcdOverlayController
{
public:
  // Lazily creates (and caches) the UndistortHelper for a frame size; walk's
  // preview session owns the helper because undistortion is applied to the
  // displayed frame independently of the overlay.
  using EnsureUndistortHelper =
    std::function<core::image::UndistortHelper *(std::uint32_t, std::uint32_t)>;

  // `pcd_topics` are the bag's non-empty PointCloud2 topics (the picker
  // candidates). `status` is the shared UI status row: picker cancellations,
  // initialization failures, and projection errors are reported there.
  PcdOverlayController(
    std::filesystem::path input_path, const io::BagReader & reader,
    std::vector<std::string> pcd_topics, std::string & status)
  : input_path_(std::move(input_path)),
    reader_(reader),
    pcd_topics_(std::move(pcd_topics)),
    status_(status)
  {
  }

  [[nodiscard]] PcdOverlayState & state() noexcept { return pcd_; }
  [[nodiscard]] const PcdOverlayState & state() const noexcept { return pcd_; }

  // Interactive checkbox picker over the candidate topics. Returns the
  // selected topics, or std::nullopt when the user cancelled or confirmed an
  // unchanged selection (both leave the current overlay untouched).
  // `backend` selects the graphics-clear protocol for the prompt redraws.
  [[nodiscard]] std::optional<std::vector<std::string>> prompt_for_topics(
    core::tui::image::ImageBackend backend);

  // One-shot initialization for a picked topic set: loads the TF buffer
  // (once) and scans every topic for timestamps and per-property ranges.
  // On failure sets `status` and returns false, leaving the previous
  // overlay state untouched.
  bool initialize(const std::vector<std::string> & topics);

  // Project the fetched point clouds onto `raster` when the overlay is
  // enabled and initialized. `record_stamp_ns` is the walked message's bag
  // record time (the frame-match clock for topics without header stamps);
  // `undistort_enabled` selects projection onto the rectified vs raw image.
  void maybe_overlay(
    core::image::PackedRaster * raster, std::int64_t record_stamp_ns,
    const std::optional<core::image::CameraInfo> & camera_info,
    const EnsureUndistortHelper & ensure_helper, bool undistort_enabled);

  // [f]: distance -> intensity (when present) -> x -> y -> z -> distance.
  void cycle_property();
  // [c]: jet -> viridis -> turbo -> plasma -> inferno -> magma -> rainbow.
  void cycle_scheme();
  // [r]: switch to a manually prompted range, or back to auto.
  void prompt_for_range(core::tui::ScrollablePager & pager, core::tui::image::ImageBackend backend);

private:
  std::filesystem::path input_path_;
  const io::BagReader & reader_;
  std::vector<std::string> pcd_topics_;
  std::string & status_;
  PcdOverlayState pcd_;
  std::optional<tf2::BufferCore> tf_buffer_;
  std::vector<core::pointcloud::PointCloudFetcher> pcd_fetchers_;
  // Parallel to pcd_fetchers_: whether each topic can be matched by capture
  // time (every cloud carried a header.stamp). Topics that can't are matched
  // by record time on both sides so the overlay stays in one clock.
  std::vector<bool> pcd_topic_has_stamps_;
};

}  // namespace bagwiz::commands

#endif  // COMMANDS__WALK_OVERLAY_HPP_
