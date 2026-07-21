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
#include "walk_overlay_scan.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

// Point-cloud projection overlay of `bagwiz walk`'s image preview: the
// overlay state, the interactive topic picker, the background initialization
// (one combined bag scan on a worker thread), and the per-frame projection.
// Moved out of walk.cpp verbatim; the interactive parts stay TTY-coupled by
// design.
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
  // Running min/max of every colour property, expanded from each cloud as it
  // is first displayed. Computing these up front would require parsing every
  // cloud in the bag; the running variant converges after the first frames
  // and keeps [f] property switches free of bag re-reads.
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

  // Progress of a start_initialize() worker.
  enum class InitState { kIdle, kRunning, kSucceeded, kFailed };

  // `pcd_topics` are the bag's PointCloud2 topics (the picker candidates).
  // They are not pre-filtered by message count — that count can require a
  // full bag scan — so an empty topic fails the initialization scan with a
  // "has no messages" status instead. `status` is the shared UI status row:
  // picker cancellations, initialization failures, and projection errors are
  // reported there.
  PcdOverlayController(
    std::filesystem::path input_path, const io::BagReader & reader,
    std::vector<std::string> pcd_topics, std::string & status)
  : input_path_(std::move(input_path)),
    reader_(reader),
    pcd_topics_(std::move(pcd_topics)),
    status_(status)
  {
  }

  // Cancels and joins any in-flight initialization worker.
  ~PcdOverlayController();

  PcdOverlayController(const PcdOverlayController &) = delete;
  PcdOverlayController & operator=(const PcdOverlayController &) = delete;
  PcdOverlayController(PcdOverlayController &&) = delete;
  PcdOverlayController & operator=(PcdOverlayController &&) = delete;

  [[nodiscard]] PcdOverlayState & state() noexcept { return pcd_; }
  [[nodiscard]] const PcdOverlayState & state() const noexcept { return pcd_; }

  // Interactive checkbox picker over the candidate topics. Returns the
  // selected topics, or std::nullopt when the user cancelled or confirmed an
  // unchanged selection (both leave the current overlay untouched).
  // `backend` selects the graphics-clear protocol for the prompt redraws.
  [[nodiscard]] std::optional<std::vector<std::string>> prompt_for_topics(
    core::tui::image::ImageBackend backend);

  // Start the overlay initialization on a worker thread: one combined bag
  // scan that decodes the TF topics and collects the selected topics' cloud
  // timestamps (see walk_overlay_scan). Returns false synchronously — with
  // `status` set — when a selected topic is not PointCloud2 or the bag has
  // no TF topic; otherwise returns true and the load proceeds in the
  // background. An in-flight load is cancelled and replaced.
  bool start_initialize(const std::vector<std::string> & topics);

  // True while a start_initialize() worker is in flight.
  [[nodiscard]] bool is_loading() const
  {
    return state_.load(std::memory_order_acquire) == InitState::kRunning;
  }
  // Whole-percent progress of the in-flight load (0 while idle).
  [[nodiscard]] int load_percent() const { return percent_.load(std::memory_order_relaxed); }

  // Reap a finished worker. kRunning: still in flight. kSucceeded: results
  // are applied — fetchers and TF buffer installed, the overlay enabled —
  // and the state returns to kIdle. kFailed: `status` carries the reason.
  // kIdle: no load is active.
  InitState poll_initialize();

  // Project the fetched point clouds onto `raster` when the overlay is
  // enabled and initialized. `record_stamp_ns` is the walked message's bag
  // record time (the frame-match clock; clouds are matched by bag record
  // time). `undistort_enabled` selects projection onto the rectified vs raw
  // image.
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
  std::vector<core::pointcloud::PointCloudFetcher> pcd_fetchers_;
  // Parallel to pcd_fetchers_: the last cloud of each fetcher already folded
  // into pcd_.ranges (identified by its cache address, which is stable until
  // the fetcher's next load), so repeated display of the same cloud does not
  // re-accumulate.
  std::vector<const core::pointcloud::PointCloud2 *> ranged_clouds_;

  // Initialization worker state. Results are heap-allocated because
  // tf2::BufferCore is immovable: the worker writes scan_result_ while
  // active_scan_ keeps serving the currently enabled overlay; on success the
  // pointers swap roles (move-assign), on failure scan_result_ is dropped
  // and the previous overlay stays intact. The UI thread consumes results
  // only after the worker reports a terminal state (the acquire/release
  // pairing on state_ orders the handoff).
  std::thread worker_;
  std::unique_ptr<OverlayScanResult> scan_result_;
  std::unique_ptr<OverlayScanResult> active_scan_;
  std::vector<std::string> pcd_topics_selected_;
  std::atomic<InitState> state_{InitState::kIdle};
  std::atomic<bool> cancel_{false};
  std::atomic<int> percent_{0};
};

}  // namespace bagwiz::commands

#endif  // COMMANDS__WALK_OVERLAY_HPP_
