// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "walk_overlay.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/core/base/terminal_input.hpp"
#include "bagwiz/core/pointcloud/overlay.hpp"
#include "bagwiz/core/pointcloud/projector.hpp"
#include "bagwiz/core/pointcloud/projector_helpers.hpp"
#include "bagwiz/core/tf/tf_buffer_loader.hpp"
#include "bagwiz/core/tui/image/terminal_image_renderer.hpp"
#include "bagwiz/core/tui/layout.hpp"
#include "bagwiz/core/tui/renderer.hpp"

#include <fmt/core.h>
#include <fmt/ostream.h>

#include <algorithm>
#include <iostream>
#include <istream>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{

constexpr const char * kPointCloudType = "sensor_msgs/msg/PointCloud2";

}  // namespace

std::string_view pcd_property_name(core::pointcloud::PointCloudProperty prop)
{
  switch (prop) {
    case core::pointcloud::PointCloudProperty::kX:
      return "x";
    case core::pointcloud::PointCloudProperty::kY:
      return "y";
    case core::pointcloud::PointCloudProperty::kZ:
      return "z";
    case core::pointcloud::PointCloudProperty::kDistance:
      return "distance";
    case core::pointcloud::PointCloudProperty::kIntensity:
      return "intensity";
  }
  return "?";
}

std::string_view pcd_scheme_name(core::pointcloud::ColorScheme s)
{
  switch (s) {
    case core::pointcloud::ColorScheme::kViridis:
      return "viridis";
    case core::pointcloud::ColorScheme::kTurbo:
      return "turbo";
    case core::pointcloud::ColorScheme::kJet:
      return "jet";
    case core::pointcloud::ColorScheme::kPlasma:
      return "plasma";
    case core::pointcloud::ColorScheme::kInferno:
      return "inferno";
    case core::pointcloud::ColorScheme::kMagma:
      return "magma";
    case core::pointcloud::ColorScheme::kRainbow:
      return "rainbow";
  }
  return "?";
}

std::optional<std::vector<std::string>> PcdOverlayController::prompt_for_topics(
  core::tui::image::ImageBackend backend)
{
  if (pcd_topics_.empty()) {
    status_ = "no non-empty PointCloud2 topics in bag";
    return std::nullopt;
  }

  // Pre-check the topics that are currently active so the picker reflects
  // the existing selection instead of resetting every topic to unchecked.
  std::vector<bool> checked(pcd_topics_.size(), false);
  for (std::size_t i = 0; i < pcd_topics_.size(); ++i) {
    checked[i] =
      std::find(pcd_.topics.begin(), pcd_.topics.end(), pcd_topics_[i]) != pcd_.topics.end();
  }
  std::size_t cursor = 0;
  bool done = false;
  bool cancelled = false;

  while (!done) {
    core::tui::image::clear_image(std::cout, backend);
    std::cout << "\x1B[2J";
    const auto term = core::tui::query_terminal_size();
    core::tui::draw_line(
      std::cout, 1,
      "  Select PointCloud2 topics (Space toggle, Enter confirm, Esc/q cancel):", term.cols);
    for (std::size_t i = 0; i < pcd_topics_.size(); ++i) {
      const std::string marker = (i == cursor) ? ">" : " ";
      const std::string box = checked[i] ? "[x]" : "[ ]";
      core::tui::draw_line(
        std::cout, static_cast<int>(i) + 3, fmt::format("  {} {} {}", marker, box, pcd_topics_[i]),
        term.cols);
    }
    std::cout.flush();

    switch (core::read_key_event()) {
      case core::KeyEvent::kScrollUp:
        if (cursor > 0) {
          --cursor;
        }
        break;
      case core::KeyEvent::kScrollDown:
        if (cursor + 1 < pcd_topics_.size()) {
          ++cursor;
        }
        break;
      case core::KeyEvent::kFirst:
        cursor = 0;
        break;
      case core::KeyEvent::kLast:
        cursor = pcd_topics_.size() - 1;
        break;
      case core::KeyEvent::kNext:
        checked[cursor] = !checked[cursor];
        break;
      case core::KeyEvent::kConfirm:
        done = true;
        break;
      case core::KeyEvent::kQuit:
        done = true;
        cancelled = true;
        break;
      case core::KeyEvent::kResize:
      default:
        break;
    }
  }

  if (cancelled) {
    status_ = "(topic selection cancelled)";
    return std::nullopt;
  }

  std::vector<std::string> selected;
  for (std::size_t i = 0; i < pcd_topics_.size(); ++i) {
    if (checked[i]) {
      selected.push_back(pcd_topics_[i]);
    }
  }

  // Confirming a selection identical to what is already applied would kick
  // off a full (slow) bag re-scan for no visible change, so short-circuit it
  // exactly like Esc/cancel — the overlay on screen is already correct.
  auto sorted = [](std::vector<std::string> v) {
    std::sort(v.begin(), v.end());
    return v;
  };
  if (!pcd_.topics.empty() && sorted(selected) == sorted(pcd_.topics)) {
    status_ = "(topic selection unchanged)";
    return std::nullopt;
  }
  return selected;
}

bool PcdOverlayController::initialize(const std::vector<std::string> & topics)
{
  for (const auto & topic : topics) {
    bool valid = false;
    for (const auto & t : reader_.topics()) {
      if (t.name == topic && t.type == kPointCloudType) {
        valid = true;
        break;
      }
    }
    if (!valid) {
      status_ = fmt::format("not a PointCloud2 topic: {}", topic);
      return false;
    }
  }

  if (!tf_buffer_.has_value()) {
    tf_buffer_.emplace();
    if (const auto err = core::load_tf_buffer(input_path_, *tf_buffer_); err.has_value()) {
      status_ = *err;
      tf_buffer_.reset();
      return false;
    }
  }

  // One pass per topic captures the timestamps *and* the min/max of every
  // colour property, so later property switches ([f]) never touch the bag.
  core::pointcloud::PropertyRanges merged_ranges;
  std::vector<std::string> initialized_topics;
  std::vector<core::pointcloud::PointCloudFetcher> new_fetchers;
  std::vector<bool> new_topic_has_stamps;

  for (const auto & topic : topics) {
    std::string error;
    auto scan = core::pointcloud::scan_point_cloud(input_path_, topic, error);
    if (!scan.has_value()) {
      status_ = error;
      return false;
    }
    merged_ranges.merge(scan->ranges);
    new_topic_has_stamps.push_back(scan->header_stamps_present);
    initialized_topics.push_back(topic);
    new_fetchers.emplace_back(input_path_, topic, std::move(scan->entries));
  }

  const auto range = merged_ranges.resolve(pcd_.property);
  pcd_.topics = std::move(initialized_topics);
  pcd_.has_intensity = merged_ranges.has_intensity;
  pcd_.ranges = merged_ranges;
  pcd_.computed_min = range.first;
  pcd_.computed_max = range.second;
  pcd_fetchers_ = std::move(new_fetchers);
  pcd_topic_has_stamps_ = std::move(new_topic_has_stamps);
  return true;
}

void PcdOverlayController::maybe_overlay(
  core::image::PackedRaster * raster, std::int64_t record_stamp_ns,
  const std::optional<core::image::CameraInfo> & camera_info,
  const EnsureUndistortHelper & ensure_helper, bool undistort_enabled)
{
  if (raster == nullptr || !pcd_.enabled || pcd_.topics.empty() || pcd_fetchers_.empty()) {
    return;
  }
  if (!camera_info.has_value()) {
    status_ = "pcd projection requires camera_info";
    return;
  }

  const auto & img = *raster;
  const auto * helper = ensure_helper(img.width, img.height);
  if (helper == nullptr) {
    status_ = "pcd projection requires camera_info";
    return;
  }
  const auto effective_ci = helper->effective_camera_info();

  std::vector<core::pointcloud::ProjectedPoint> all_points;
  std::string last_error;
  for (std::size_t i = 0; i < pcd_fetchers_.size(); ++i) {
    // Pair the frame with the point cloud nearest in time (see
    // core::pointcloud::choose_frame_match for the clock rule). The chosen
    // target is also the TF-lookup time.
    const auto match = core::pointcloud::choose_frame_match(
      img.header_stamp_ns, record_stamp_ns, pcd_topic_has_stamps_[i]);
    const std::int64_t match_ns = match.target_ns;

    std::string error;
    const auto * cloud = pcd_fetchers_[i].fetch(match_ns, match.key, error);
    if (cloud == nullptr) {
      last_error = std::move(error);
      continue;
    }

    const auto projected = core::pointcloud::project_cloud_for_frame(
      *cloud, effective_ci, *tf_buffer_, img.width, img.height, pcd_.property,
      /*use_rectified=*/undistort_enabled, match_ns);
    if (!projected.ok()) {
      last_error = std::move(projected.error);
      continue;
    }
    all_points.insert(all_points.end(), projected.points.begin(), projected.points.end());
  }

  if (all_points.empty()) {
    if (!last_error.empty()) {
      status_ = std::move(last_error);
    }
    return;
  }

  const double vmin = pcd_.auto_range ? pcd_.computed_min : pcd_.manual_min;
  const double vmax = pcd_.auto_range ? pcd_.computed_max : pcd_.manual_max;
  const auto err = core::pointcloud::overlay_projected_points(
    img, all_points, vmin, vmax, pcd_.scheme, pcd_.point_size, pcd_.alpha, *raster);
  if (!err.empty()) {
    status_ = err;
  }
}

void PcdOverlayController::cycle_property()
{
  // Cycle order: distance -> intensity (only when the cloud carries it)
  //           -> x -> y -> z -> distance ...
  auto next = [&](core::pointcloud::PointCloudProperty cur) {
    using Property = core::pointcloud::PointCloudProperty;
    switch (cur) {
      case Property::kDistance:
        return pcd_.has_intensity ? Property::kIntensity : Property::kX;
      case Property::kIntensity:
        return Property::kX;
      case Property::kX:
        return Property::kY;
      case Property::kY:
        return Property::kZ;
      case Property::kZ:
        return Property::kDistance;
    }
    return Property::kDistance;
  };
  pcd_.property = next(pcd_.property);
  // Auto range reuses the extent captured up front, so switching property is
  // O(1) and never re-reads the bag (the timestamps/fetchers are unchanged).
  if (pcd_.auto_range) {
    const auto range = pcd_.ranges.resolve(pcd_.property);
    pcd_.computed_min = range.first;
    pcd_.computed_max = range.second;
  }
}

void PcdOverlayController::cycle_scheme()
{
  switch (pcd_.scheme) {
    case core::pointcloud::ColorScheme::kJet:
      pcd_.scheme = core::pointcloud::ColorScheme::kViridis;
      break;
    case core::pointcloud::ColorScheme::kViridis:
      pcd_.scheme = core::pointcloud::ColorScheme::kTurbo;
      break;
    case core::pointcloud::ColorScheme::kTurbo:
      pcd_.scheme = core::pointcloud::ColorScheme::kPlasma;
      break;
    case core::pointcloud::ColorScheme::kPlasma:
      pcd_.scheme = core::pointcloud::ColorScheme::kInferno;
      break;
    case core::pointcloud::ColorScheme::kInferno:
      pcd_.scheme = core::pointcloud::ColorScheme::kMagma;
      break;
    case core::pointcloud::ColorScheme::kMagma:
      pcd_.scheme = core::pointcloud::ColorScheme::kRainbow;
      break;
    case core::pointcloud::ColorScheme::kRainbow:
      pcd_.scheme = core::pointcloud::ColorScheme::kJet;
      break;
  }
}

void PcdOverlayController::prompt_for_range(
  core::tui::ScrollablePager & pager, core::tui::image::ImageBackend backend)
{
  if (pcd_.auto_range) {
    pcd_.auto_range = false;
    core::tui::image::clear_image(std::cout, backend);
    std::cout << "\x1B[2J";
    pager.with_line_input([&](std::istream & in, std::ostream & out) {
      out << "Manual min: ";
      out.flush();
      std::string line;
      if (std::getline(in, line)) {
        try {
          pcd_.manual_min = std::stod(line);
        } catch (...) {
        }
      }
      out << "Manual max: ";
      out.flush();
      if (std::getline(in, line)) {
        try {
          pcd_.manual_max = std::stod(line);
        } catch (...) {
        }
      }
    });
  } else {
    pcd_.auto_range = true;
  }
}

}  // namespace bagwiz::commands
