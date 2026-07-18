// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "walk_preview.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/core/base/terminal_input.hpp"
#include "bagwiz/core/image/image_encoder.hpp"
#include "bagwiz/core/tui/image/terminal_image_renderer.hpp"
#include "bagwiz/core/tui/renderer.hpp"
#include "bagwiz/core/tui/width.hpp"
#include "walk_frame.hpp"  // NOLINT(build/include_subdir) src-local shared header
#include "walk_save.hpp"   // NOLINT(build/include_subdir) src-local shared header

#include <fmt/core.h>

#include <algorithm>
#include <exception>
#include <filesystem>
#include <iostream>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{

// Upper bound on decoded preview frames kept in memory at once. Decoding a
// frame (JPEG/PNG via libav, or a raw copy) dominates repaint cost, so we cache
// recently viewed rasters; the cap bounds memory (each raster is
// width * height * 3 bytes, so a handful of HD frames is a few tens of MB).
constexpr std::size_t kPreviewCacheCapacity = 16;

}  // namespace

ImagePreviewSession::ImagePreviewSession(
  MessageCursor & cursor, PcdOverlayController & overlay, core::tui::ScrollablePager & pager,
  std::string & status, std::string topic_name, std::string type_name,
  core::tui::image::TerminalImageCaps image_caps,
  const std::optional<core::image::CameraInfo> & camera_info, const std::string & camera_info_error)
: cursor_(cursor),
  overlay_(overlay),
  pager_(pager),
  status_(status),
  topic_name_(std::move(topic_name)),
  type_name_(std::move(type_name)),
  image_caps_(image_caps),
  camera_info_(camera_info),
  camera_info_error_(camera_info_error),
  decoded_frames_(kPreviewCacheCapacity)
{
}

core::image::UndistortHelper * ImagePreviewSession::ensure_undistort_helper(
  std::uint32_t w, std::uint32_t h)
{
  if (!camera_info_.has_value()) {
    return nullptr;
  }
  if (!undistort_helper_ || undistort_helper_w_ != w || undistort_helper_h_ != h) {
    undistort_helper_ = std::make_unique<core::image::UndistortHelper>(*camera_info_, w, h);
    undistort_helper_w_ = w;
    undistort_helper_h_ = h;
  }
  return undistort_helper_.get();
}

void ImagePreviewSession::maybe_undistort(core::image::PackedRaster * raster)
{
  if (raster == nullptr) {
    return;
  }
  auto * helper = ensure_undistort_helper(raster->width, raster->height);
  if (helper == nullptr) {
    return;
  }
  const auto remapped = helper->remap(raster->bgr, raster->width * 3);
  raster->bgr.assign(remapped.begin(), remapped.end());
  raster->encoding = "bgr8";
}

core::image::PackedRasterResult ImagePreviewSession::compose_frame(std::size_t idx)
{
  core::image::PackedRasterResult pr;
  const auto & msg = cursor_.cache()[idx];
  auto hit = decoded_frames_.get(idx, type_name_, msg.payload);
  if (hit.raster == nullptr) {
    pr.error = std::move(hit.error);
    return pr;
  }
  pr.raster = *hit.raster;  // copy the pristine base before mutating overlays
  if (undistort_enabled_) {
    maybe_undistort(&*pr.raster);
  }
  if (overlay_.state().enabled) {
    overlay_.maybe_overlay(
      &*pr.raster, cursor_.cache()[cursor_.index()].timestamp_ns, camera_info_,
      [this](std::uint32_t w, std::uint32_t h) { return ensure_undistort_helper(w, h); },
      undistort_enabled_);
  }
  return pr;
}

void ImagePreviewSession::render(std::ostream & out, core::tui::Size term)
{
  const std::size_t index = cursor_.index();
  const auto & cache = cursor_.cache();
  const bool exhausted = cursor_.exhausted();
  const auto & pcd = overlay_.state();
  const auto & status = status_;
  const auto & image_caps = image_caps_;
  const auto & topic_name = topic_name_;

  const int rows = std::max(1, term.rows);
  const int cols = std::max(1, term.cols);

  // Bracket the whole repaint in a synchronized update so the terminal keeps
  // showing the current frame until the new one is fully transmitted, then
  // swaps atomically. Without this the clear below blanks the screen for as
  // long as the terminal needs to receive and decode the next image, which
  // reads as a one-frame "blink" on every prev/next. Unsupported terminals
  // ignore the mode and behave exactly as before.
  core::tui::begin_synchronized_update(out);

  // Drop any previously transmitted graphics and wipe the screen so kitty
  // placements do not accumulate across navigation/resize.
  core::tui::image::clear_image(out, image_caps.backend);
  out << "\x1B[2J";

  const char * total_suffix = exhausted ? "" : "+";
  const std::size_t last_loaded_index = cache.size() - 1;
  auto pr = compose_frame(index);

  std::string info;
  if (pr.ok()) {
    const auto & img = *pr.raster;
    info = fmt::format(
      "  {}x{}   [{} / {}{}]", img.width, img.height, index, last_loaded_index, total_suffix);
  } else {
    info = fmt::format("  [{} / {}{}]", index, last_loaded_index, total_suffix);
  }
  // Surface the save outcome (or any transient message) on the info row;
  // navigate() clears `status` on a cursor move, so it disappears as soon as
  // the user pages to another frame.
  if (!status.empty()) {
    info += fmt::format("   {}", status);
  }
  // Every state field reads as "field: value" with the value emphasised so
  // it stands out from the label. The SGR wrapper is zero display-width (see
  // width.cpp), so it does not perturb the wrap/truncate accounting below.
  auto hl = [](auto && value) { return fmt::format("\x1B[1;36m{}\x1B[0m", value); };

  info += fmt::format("   undistort: {}", hl(undistort_enabled_ ? "on" : "off"));
  if (!pcd.topics.empty()) {
    const std::string range_text =
      pcd.auto_range ? "auto" : fmt::format("{:.2f}-{:.2f}", pcd.manual_min, pcd.manual_max);
    info += fmt::format(
      "   pcd: {}   property: {}   range: {}   scheme: {}   size: {}   alpha: {}",
      hl(pcd.enabled ? "on" : "off"), hl(pcd_property_name(pcd.property)), hl(range_text),
      hl(pcd_scheme_name(pcd.scheme)), hl(pcd.point_size), hl(fmt::format("{:.1f}", pcd.alpha)));
  }

  // Header: the topic/type row and the info row, each wrapped to width the
  // same way the YAML view's header and the legend below are, so a narrow
  // terminal shows the full text on continuation lines instead of truncating
  // it at the right edge. The image region starts just below the wrapped
  // header (see region_row).
  std::vector<std::string> header_lines;
  append_wrapped(header_lines, fmt::format("  {}", topic_name), cols);
  append_wrapped(header_lines, info, cols);
  for (std::size_t i = 0; i < header_lines.size(); ++i) {
    core::tui::draw_line(out, 1 + static_cast<int>(i), header_lines[i], cols);
  }

  // Wrap the key legend the way the YAML footer does, so a narrow terminal
  // shows every key on continuation lines instead of truncating the row. The
  // wrapped legend is pinned to the bottom and the image region above shrinks
  // to make room, mirroring how the YAML view derives its body height from
  // the wrapped footer.
  // The pcd overlay adjustment keys (f/c/r/=/-/[/]) are only meaningful once
  // a PointCloud2 topic is selected, so surface them in the legend under the
  // same condition the info row uses to show pcd state. The toggle/select
  // keys ([p]/[t]) stay visible unconditionally to guide the user to enable
  // the overlay in the first place.
  std::string legend_text =
    "  [→ / Space] next   [← / b] prev   [,] -1s   [.] +1s   [<] -10s   [>] +10s   [g] first "
    "  [G] last   [s] save   "
    "[u] undistort   [p] project pcd   [t] select pcd topics";
  if (!pcd.topics.empty()) {
    legend_text += "   [f] property   [c] scheme   [r] range   [= / -] size   [ [ / ] ] alpha";
  }
  legend_text += "   [q] back";
  const std::vector<std::string> legend_lines = core::tui::wrap_to_width(legend_text, cols);
  const int legend_top = std::max(1, rows - static_cast<int>(legend_lines.size()) + 1);

  // Image region: from the row just below the wrapped header down to the row
  // above the first legend line.
  const int region_row = 1 + static_cast<int>(header_lines.size());
  const int region_rows = std::max(1, legend_top - region_row);
  if (pr.ok()) {
    core::tui::image::CellRegion region;
    region.row = region_row;
    region.col = 1;
    region.rows = region_rows;
    region.cols = cols;
    const std::string err = core::tui::image::render_image(out, *pr.raster, region, image_caps);
    if (!err.empty()) {
      core::tui::draw_line(out, region_row, fmt::format("  preview unavailable: {}", err), cols);
    }
  } else {
    core::tui::draw_line(
      out, region_row, fmt::format("  cannot decode this message: {}", pr.error), cols);
  }

  for (std::size_t i = 0; i < legend_lines.size(); ++i) {
    core::tui::draw_line(out, legend_top + static_cast<int>(i), legend_lines[i], cols);
  }

  // Close the synchronized update: the terminal now reveals the fully
  // assembled frame in one atomic swap.
  core::tui::end_synchronized_update(out);
  out.flush();
}

void ImagePreviewSession::save_image()
{
  const std::size_t index = cursor_.index();
  const auto & topic_name = topic_name_;
  const auto & image_caps = image_caps_;

  status_.clear();
  auto pr = compose_frame(index);
  if (!pr.ok()) {
    status_ = fmt::format("cannot save: {}", pr.error);
    return;
  }
  const auto encoded = core::image::encode_png(*pr.raster);
  if (!encoded.ok()) {
    status_ = fmt::format("cannot save: {}", encoded.error);
    return;
  }

  const std::string default_base = fmt::format("{}_{}.png", topic_for_filename(topic_name), index);
  std::filesystem::path cwd;
  try {
    cwd = std::filesystem::current_path();
  } catch (const std::exception & e) {
    status_ = fmt::format("cannot resolve working directory: {}", e.what());
    return;
  }

  // Drop the on-screen graphic before switching to cooked-mode line input so
  // the prompt is not drawn over a kitty placement; run() repaints the frame
  // afterward.
  core::tui::image::clear_image(std::cout, image_caps.backend);
  std::cout << "\x1B[2J";
  std::cout.flush();

  const auto & bytes = *encoded.png;
  save_bytes_with_prompt(
    pager_, "Save image path", cwd, default_base,
    std::span<const std::byte>(bytes.data(), bytes.size()), status_);
}

void ImagePreviewSession::run()
{
  auto & pcd = overlay_.state();
  const auto & camera_info = camera_info_;
  const auto & camera_info_error = camera_info_error_;
  const auto & image_caps = image_caps_;

  std::ostream & out = std::cout;
  bool running = true;
  bool needs_render = true;
  while (running) {
    if (needs_render) {
      render(out, core::tui::query_terminal_size());
      needs_render = false;
    }
    switch (core::read_key_event()) {
      case core::KeyEvent::kNext:
        // Re-decode only when the cursor actually moved; otherwise the frame
        // is unchanged and a full decode + scale would be wasted.
        needs_render = cursor_.navigate(MsgNav::kNext);
        break;
      case core::KeyEvent::kPrev:
        needs_render = cursor_.navigate(MsgNav::kPrev);
        break;
      case core::KeyEvent::kFirst:
        needs_render = cursor_.navigate(MsgNav::kFirst);
        break;
      case core::KeyEvent::kLast:
        needs_render = cursor_.navigate(MsgNav::kLast);
        break;
      case core::KeyEvent::kStepForward1s:
        needs_render = cursor_.navigate(MsgNav::kStepForward1s);
        break;
      case core::KeyEvent::kStepForward10s:
        needs_render = cursor_.navigate(MsgNav::kStepForward10s);
        break;
      case core::KeyEvent::kStepBackward1s:
        needs_render = cursor_.navigate(MsgNav::kStepBackward1s);
        break;
      case core::KeyEvent::kStepBackward10s:
        needs_render = cursor_.navigate(MsgNav::kStepBackward10s);
        break;
      case core::KeyEvent::kResize:
        needs_render = true;  // geometry changed: re-fit and re-render
        break;
      case core::KeyEvent::kSaveYaml:
        // In the preview, [s] saves the displayed frame as a PNG (the YAML
        // view's [s] still saves YAML). Always repaint so the save status is
        // shown and the prompt's screen clear is undone.
        save_image();
        needs_render = true;
        break;
      case core::KeyEvent::kToggleUndistort:
        // Toggling undistort also re-aims the pcd overlay: with undistort on
        // points project onto the rectified image, with it off they project
        // onto the raw image using the lens distortion (see maybe_overlay).
        if (!camera_info.has_value()) {
          status_ = camera_info_error.empty() ? "undistort: no camera_info"
                                              : "undistort: " + camera_info_error;
        } else {
          undistort_enabled_ = !undistort_enabled_;
        }
        needs_render = true;
        break;
      case core::KeyEvent::kToggleProjectPcd:
        if (!camera_info.has_value()) {
          status_ = camera_info_error.empty() ? "pcd: no camera_info" : "pcd: " + camera_info_error;
        } else if (pcd.topics.empty()) {
          if (auto topics = overlay_.prompt_for_topics(image_caps.backend);
              topics.has_value() && !topics->empty()) {
            if (overlay_.initialize(*topics)) {
              pcd.enabled = true;
            }
          }
        } else {
          pcd.enabled = !pcd.enabled;
        }
        needs_render = true;
        break;
      case core::KeyEvent::kSelectPcdTopic:
        if (camera_info.has_value()) {
          if (auto topics = overlay_.prompt_for_topics(image_caps.backend); topics.has_value()) {
            if (topics->empty()) {
              pcd.enabled = false;
            } else if (overlay_.initialize(*topics)) {
              pcd.enabled = true;
            }
          }
        } else {
          status_ = camera_info_error.empty() ? "pcd: no camera_info" : "pcd: " + camera_info_error;
        }
        needs_render = true;
        break;
      case core::KeyEvent::kCyclePcdProperty:
        overlay_.cycle_property();
        needs_render = true;
        break;
      case core::KeyEvent::kCyclePcdScheme:
        overlay_.cycle_scheme();
        needs_render = true;
        break;
      case core::KeyEvent::kTogglePcdRange:
        overlay_.prompt_for_range(pager_, image_caps.backend);
        needs_render = true;
        break;
      case core::KeyEvent::kPcdPointSizeUp:
        pcd.point_size = std::min(pcd.point_size + 1, 64U);
        needs_render = true;
        break;
      case core::KeyEvent::kPcdPointSizeDown:
        pcd.point_size = std::max(pcd.point_size - 1, 1U);
        needs_render = true;
        break;
      case core::KeyEvent::kPcdAlphaUp:
        pcd.alpha = std::min(pcd.alpha + 0.1f, 1.0f);
        needs_render = true;
        break;
      case core::KeyEvent::kPcdAlphaDown:
        pcd.alpha = std::max(pcd.alpha - 0.1f, 0.0f);
        needs_render = true;
        break;
      case core::KeyEvent::kQuit:
        running = false;
        break;
      default:
        break;  // scroll / expand keys are inert in the preview
    }
  }
  // Hand a clean screen back to the pager for the YAML repaint.
  core::tui::image::clear_image(out, image_caps.backend);
  out << "\x1B[2J";
  out.flush();
}

}  // namespace bagwiz::commands
