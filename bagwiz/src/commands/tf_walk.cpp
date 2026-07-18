// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/tf_walk.hpp"

#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/base/str_utils.hpp"
#include "bagwiz/core/base/terminal_input.hpp"
#include "bagwiz/core/tf/tf_buffer_loader.hpp"
#include "bagwiz/core/tf/tf_chain.hpp"
#include "bagwiz/core/tf/tf_topics.hpp"
#include "bagwiz/core/tf/tf_transform_format.hpp"
#include "bagwiz/core/tf/tf_walk_timeline.hpp"
#include "bagwiz/core/tui/layout.hpp"
#include "bagwiz/core/tui/pager.hpp"
#include "bagwiz/core/tui/width.hpp"
#include "bagwiz/io/bag_io.hpp"
#include "bagwiz/io/bag_open.hpp"

#include <tf2/buffer_core.hpp>
#include <tf2/time.hpp>

#include <fmt/core.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{

constexpr const char * kLogger = "bagwiz.cmd.tf.walk";
// Retain every dynamic transform for the whole walk so stepping back to an
// early time still resolves. A year dwarfs any realistic bag and matches
// `tf static calc`'s buffer sizing.
constexpr std::chrono::hours kTfBufferCacheTime{24 * 365};

std::int64_t time_point_to_ns(tf2::TimePoint tp)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch()).count();
}

// Join frame ids with ", "; returns "(none)" for an empty list.
std::string join_frames(std::vector<std::string> frames)
{
  std::sort(frames.begin(), frames.end());
  std::string csv;
  for (std::size_t i = 0; i < frames.size(); ++i) {
    if (i > 0) {
      csv += ", ";
    }
    csv += frames[i];
  }
  return csv.empty() ? "(none)" : csv;
}

}  // namespace

int run_tf_walk(
  const std::filesystem::path & input_path, const std::string & of_frame,
  const std::string & ref_frame)
{
  if (!::isatty(STDIN_FILENO) || !::isatty(STDOUT_FILENO)) {
    BAGWIZ_LOG_ERROR(
      kLogger, "tf walk requires an interactive terminal (stdin+stdout must be TTY)");
    return 1;
  }

  auto reader = io::open_read_or_log(input_path, kLogger);
  if (!reader) {
    return 1;
  }

  // Merge every TF topic (no static/dynamic filtering); static topics are still
  // fed as static so the buffer can resolve them at any query time.
  const std::vector<core::TfTopic> tf_topics = core::collect_tf_topics(*reader);
  if (tf_topics.empty()) {
    BAGWIZ_LOG_ERROR(kLogger, "Bag has no tf2_msgs/msg/TFMessage topic; nothing to walk.");
    return 1;
  }

  tf2::BufferCore buffer{kTfBufferCacheTime};
  std::vector<tf2::TimePoint> stamps;
  try {
    core::TfReplayOutputs outputs;
    outputs.buffer = &buffer;
    outputs.stamps = &stamps;
    core::replay_tf_topics(*reader, tf_topics, outputs);
  } catch (const std::exception & e) {
    BAGWIZ_LOG_ERROR(kLogger, "Failed to load TF from the bag: %s", e.what());
    return 1;
  }

  const std::vector<tf2::TimePoint> timeline = core::build_tf_walk_timeline(std::move(stamps));
  if (timeline.empty()) {
    BAGWIZ_LOG_ERROR(kLogger, "TF topics carry no decodable transforms; nothing to walk.");
    return 1;
  }

  // Reject an --of/--ref the bag's TF tree does not contain before opening the
  // viewer. tf2's lookupTransform returns an identity transform when
  // target == source WITHOUT checking the frame exists, so
  // `tf walk --of <f> --ref <f>` for an absent frame would otherwise display a
  // bogus identity transform.
  const std::vector<std::string> missing = core::missing_frames(buffer, of_frame, ref_frame);
  if (!missing.empty()) {
    BAGWIZ_LOG_ERROR(
      kLogger, "Frame(s) not present in the bag's TF tree: %s", join_frames(missing).c_str());
    BAGWIZ_LOG_ERROR(
      kLogger, "Available frames: %s", join_frames(buffer.getAllFrameNames()).c_str());
    return 1;
  }

  std::size_t index = 0;
  std::string status;

  core::tui::PagerConfig pager_cfg;
  core::tui::ScrollablePager pager(pager_cfg);

  auto append_wrapped = [](std::vector<std::string> & out, std::string_view line, int cols) {
    for (auto & w : core::tui::wrap_to_width(line, cols)) {
      out.push_back(std::move(w));
    }
  };

  auto build_frame = [&](std::size_t scroll, core::tui::Size term) -> core::tui::Frame {
    core::tui::Frame frame;
    const int cols = std::max(1, term.cols);
    const std::size_t last_index = timeline.size() - 1;

    const auto step = core::resolve_tf_walk_step(buffer, timeline[index], of_frame, ref_frame);

    // Header: the resolved transform's timestamp, then a blank separator.
    append_wrapped(
      frame.header,
      fmt::format("timestamp: {}", core::format_timestamp(time_point_to_ns(timeline[index]))),
      cols);
    frame.header.emplace_back();

    std::vector<std::string> body_logical;
    if (step.transform.has_value()) {
      // Show the full frame chain (resolved at this step's time) in the
      // direction line, not just the endpoints. The step resolved
      // successfully, so a chain exists; fall back to the bare endpoints if
      // resolve_chain cannot reconstruct it.
      std::vector<std::string> chain =
        core::resolve_chain(buffer, of_frame, ref_frame, timeline[index]);
      if (chain.empty()) {
        chain = {of_frame, ref_frame};
      }
      body_logical = core::split_lines(core::format_transform_human(*step.transform, chain));
    } else {
      body_logical.push_back(
        fmt::format(
          "⚠  Could not resolve of={} ref={} at this time: {}", of_frame, ref_frame, step.error));
    }
    frame.body.reserve(body_logical.size());
    for (const auto & line : body_logical) {
      append_wrapped(frame.body, line, cols);
    }

    // Footer: blank separator, index row (patched with the scroll hint below),
    // the key legend, and the status row.
    std::vector<std::string> footer_logical;
    footer_logical.emplace_back();
    footer_logical.emplace_back();  // index row placeholder
    footer_logical.emplace_back(
      "  [→/Space] next   [←/b] prev   [↑/k] up   [↓/j] down   "
      "[Home/H] head   [End/T] tail   [g] first   [G] last   [q] quit");
    footer_logical.push_back(status.empty() ? std::string{} : fmt::format("  {}", status));

    std::vector<std::string> footer_wrapped;
    auto wrap_footer = [&](const std::vector<std::string> & src) {
      footer_wrapped.clear();
      for (const auto & line : src) {
        append_wrapped(footer_wrapped, line, cols);
      }
    };
    auto recompute_footer = [&](const std::string & index_line) {
      footer_logical[1] = index_line;
      wrap_footer(footer_logical);
    };

    const std::string index_no_hint =
      fmt::format("  [{} / {}]  of={}  ref={}", index, last_index, of_frame, ref_frame);
    recompute_footer(index_no_hint);

    auto body_rows_for = [&](const std::vector<std::string> & footer) {
      return std::max(
        0, term.rows - static_cast<int>(frame.header.size()) - static_cast<int>(footer.size()));
    };

    int body_rows = body_rows_for(footer_wrapped);
    const std::size_t total_body = frame.body.size();
    if (body_rows > 0 && total_body > static_cast<std::size_t>(body_rows)) {
      const std::size_t end = std::min(scroll + static_cast<std::size_t>(body_rows), total_body);
      const std::string hint = fmt::format("    lines {}-{} of {}", scroll + 1, end, total_body);
      recompute_footer(index_no_hint + hint);
      // Recomputing can change the footer height (the index row may now wrap),
      // so re-derive body_rows once and patch the hint if it changed.
      body_rows = body_rows_for(footer_wrapped);
      if (total_body > static_cast<std::size_t>(std::max(body_rows, 0))) {
        const std::size_t end2 = std::min(scroll + static_cast<std::size_t>(body_rows), total_body);
        const std::string hint2 =
          fmt::format("    lines {}-{} of {}", scroll + 1, end2, total_body);
        if (hint2 != hint) {
          recompute_footer(index_no_hint + hint2);
        }
      }
    }

    frame.footer = std::move(footer_wrapped);
    return frame;
  };

  auto on_nav = [&](core::tui::NavKey nav) -> core::tui::AppKeyResult {
    status.clear();
    switch (nav) {
      case core::tui::NavKey::kNext:
        if (index + 1 < timeline.size()) {
          ++index;
        } else {
          index = 0;
          status = "(wrapped to first)";
        }
        pager.set_scroll_offset(0);
        return core::tui::AppKeyResult::kHandled;
      case core::tui::NavKey::kPrev:
        if (index > 0) {
          --index;
          pager.set_scroll_offset(0);
        } else {
          status = "(at first transform)";
        }
        return core::tui::AppKeyResult::kHandled;
      case core::tui::NavKey::kFirst:
        index = 0;
        pager.set_scroll_offset(0);
        return core::tui::AppKeyResult::kHandled;
      case core::tui::NavKey::kLast:
        index = timeline.size() - 1;
        pager.set_scroll_offset(0);
        return core::tui::AppKeyResult::kHandled;
      case core::tui::NavKey::kResize:
        return core::tui::AppKeyResult::kHandled;
      default:
        return core::tui::AppKeyResult::kIgnored;
    }
  };

  // No app-specific keys: tf walk has no save / array-expand actions.
  auto on_app_key = [&](core::KeyEvent) -> core::tui::AppKeyResult {
    return core::tui::AppKeyResult::kIgnored;
  };

  return pager.run(build_frame, on_nav, on_app_key);
}

}  // namespace bagwiz::commands
