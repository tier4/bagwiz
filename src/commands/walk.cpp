// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/core/camera_info_resolver.hpp"
#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/image/camera_info.hpp"
#include "bagwiz/core/image/image_encoder.hpp"
#include "bagwiz/core/image/packed_raster.hpp"
#include "bagwiz/core/image/undistort.hpp"
#include "bagwiz/core/logging.hpp"
#include "bagwiz/core/message_formatter.hpp"
#include "bagwiz/core/terminal_input.hpp"
#include "bagwiz/core/tui/image/terminal_image_caps.hpp"
#include "bagwiz/core/tui/image/terminal_image_renderer.hpp"
#include "bagwiz/core/tui/layout.hpp"
#include "bagwiz/core/tui/pager.hpp"
#include "bagwiz/core/tui/renderer.hpp"
#include "bagwiz/core/tui/width.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <fmt/core.h>
#include <fmt/ostream.h>
#include <unistd.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <istream>
#include <iterator>
#include <memory>
#include <ostream>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{

constexpr const char * kLogger = "bagwiz.cmd.walk";

// Message-cursor moves shared by the YAML view and the image preview, so
// wrap-around, "at first message", and G's full-scan behave identically in both.
enum class MsgNav { kNext, kPrev, kFirst, kLast, kStepForward1s, kStepBackward1s };

constexpr int64_t kOneSecondNs = 1'000'000'000;

// Cached owning copy of a single bag message. RawMessage's span is
// invalidated by the next BagReader::next() call, so walk must take a
// copy to allow backward navigation.
struct OwnedMessage
{
  int64_t timestamp_ns = 0;
  std::vector<std::byte> payload;
};

std::string format_timestamp(int64_t ns)
{
  const auto seconds = static_cast<std::time_t>(ns / 1'000'000'000);
  const auto nanos = static_cast<int64_t>(ns % 1'000'000'000);
  std::tm tm_utc{};
  ::gmtime_r(&seconds, &tm_utc);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm_utc);
  return fmt::format("{}.{:09d} UTC ({}.{:09d})", buf, nanos, seconds, nanos);
}

std::vector<std::byte> copy_payload(std::span<const std::byte> src)
{
  return std::vector<std::byte>(src.begin(), src.end());
}

// Split a '\n'-delimited string into owned lines so the body vector
// can outlive the source string. A trailing '\n' does not produce an
// empty final element; a missing trailing '\n' keeps the tail.
std::vector<std::string> split_lines(const std::string & s)
{
  std::vector<std::string> out;
  std::size_t start = 0;
  for (std::size_t i = 0; i < s.size(); ++i) {
    if (s[i] == '\n') {
      out.emplace_back(s.data() + start, i - start);
      start = i + 1;
    }
  }
  if (start < s.size()) {
    out.emplace_back(s.data() + start, s.size() - start);
  }
  return out;
}

// ROS topic names use `/`; replace each `/` with `__` so path separators
// do not collide with underscores that appear inside topic name segments.
std::string topic_for_filename(std::string_view topic)
{
  std::string out;
  out.reserve(topic.size() * 2);
  for (unsigned char uc : topic) {
    const char c = static_cast<char>(uc);
    if (c == '/') {
      out += "__";
    } else {
      out.push_back(c);
    }
  }
  return out;
}

std::filesystem::path resolve_save_path(
  const std::string & line_from_stdin, const std::filesystem::path & cwd,
  const std::string & default_filename)
{
  std::string trimmed = line_from_stdin;
  while (!trimmed.empty() && (trimmed.back() == ' ' || trimmed.back() == '\t')) {
    trimmed.pop_back();
  }
  if (trimmed.empty()) {
    return cwd / default_filename;
  }

  std::filesystem::path user_path(trimmed);
  std::error_code ec;
  if (std::filesystem::exists(user_path, ec) && std::filesystem::is_directory(user_path, ec)) {
    return user_path / default_filename;
  }
  const char last = trimmed.back();
  if (last == '/' || last == '\\') {
    return std::filesystem::path(trimmed) / default_filename;
  }
  return user_path;
}

// Paint `text` as a rainbow by assigning each character a standard ANSI
// foreground color in sequence. The returned string contains the SGR escapes
// and a trailing reset; width-aware code treats those escapes as zero-width,
// so wrapping/layout is unaffected.
std::string rainbow_text(std::string_view text)
{
  // Red, yellow, green, cyan, blue, magenta — a classic 6-step rainbow.
  constexpr const char * kColors[] = {"\x1B[31m", "\x1B[33m", "\x1B[32m",
                                      "\x1B[36m", "\x1B[34m", "\x1B[35m"};
  std::string out;
  out.reserve(text.size() * 6);
  for (std::size_t i = 0; i < text.size(); ++i) {
    out += kColors[i % std::size(kColors)];
    out.push_back(text[i]);
  }
  out += "\x1B[0m";
  return out;
}

}  // namespace

// `bagwiz walk <input> <topic>` walks the messages of a single topic
// one at a time and renders each payload as YAML, mirroring what
// `ros2 topic echo` produces. Decoding relies on the rosidl
// introspection typesupport library (or the schema-driven path when
// the MCAP carries a `ros2msg` schema).
//
// The view is a pager driven by `bagwiz::core::tui::ScrollablePager`:
// the header (timestamp, size) and footer (index + topic/type + scroll
// hint, key legend, status row) are pinned in place,
// and only the body region scrolls.
//
// Keys:
//   right / Space : next message (wraps from last back to first)
//   left / b      : previous message
//   .             : jump forward ~1 second in time
//   ,             : jump backward ~1 second in time
//   up / k        : scroll body up one line
//   down / j      : scroll body down one line
//   Home / H      : jump body scroll to the head
//   End / T       : jump body scroll to the tail
//   g / G         : jump to first / last message (G forces a full scan)
//   s             : save current message as yaml; inside the image preview,
//                   save the displayed frame as a PNG (both prompt for a path)
//   a             : toggle full-expansion of long primitive arrays
//   i             : toggle in-terminal image preview (image topics on a
//                   Kitty/Sixel-capable terminal; absent otherwise)
//   q / Ctrl-C    : quit
// Messages are cached lazily so `prev` stays O(1) for anything already
// seen and `G` is the only key that can trigger a full-remaining scan.
class WalkCommand : public Command
{
public:
  [[nodiscard]] std::string_view name() const override { return "walk"; }
  [[nodiscard]] std::string_view description() const override
  {
    return "Walk messages of a topic as decoded YAML";
  }

  void configure(CLI::App & app) override
  {
    app.add_option("input", input_path_, "Bag path (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    app.add_option("topic", topic_, "Topic name to inspect")->required();
    app.add_option(
      "--cam-info", camera_info_topic_, "Explicit CameraInfo topic for undistort preview");
  }

  int run() override
  {
    if (!::isatty(STDIN_FILENO) || !::isatty(STDOUT_FILENO)) {
      BAGWIZ_LOG_ERROR(kLogger, "walk requires an interactive terminal (stdin+stdout must be TTY)");
      return 1;
    }

    std::unique_ptr<io::BagReader> reader;
    try {
      reader = io::open_read(input_path_);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to open %s: %s", input_path_.c_str(), e.what());
      return 1;
    }

    const io::TopicInfo * topic_info = nullptr;
    for (const auto & t : reader->topics()) {
      if (t.name == topic_) {
        topic_info = &t;
        break;
      }
    }
    if (topic_info == nullptr) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Topic '%s' is not present in %s", topic_.c_str(), input_path_.c_str());
      return 1;
    }

    std::optional<std::string> camera_info_topic;
    std::optional<core::image::CameraInfo> camera_info;
    std::string camera_info_error;

    if (camera_info_topic_.has_value()) {
      if (const auto err =
            core::camera_info::validate_camera_info_topic(input_path_, *camera_info_topic_);
          err.has_value()) {
        camera_info_error = *err;
      } else {
        camera_info_topic = *camera_info_topic_;
      }
    } else {
      const auto resolution =
        core::camera_info::resolve_camera_info_topic(topic_, reader->topics());
      camera_info_topic = resolution.topic;
      if (resolution.error.has_value()) {
        camera_info_error = *resolution.error;
      }
    }

    if (camera_info_topic.has_value()) {
      const auto ci = core::camera_info::load_camera_info(input_path_, *camera_info_topic);
      if (ci.ok()) {
        camera_info = *ci.info;
      } else {
        camera_info_error = ci.error;
      }
    }

    io::ReadFilter read_filter;
    read_filter.topics.push_back(topic_);
    reader->set_filter(read_filter);

    const std::string topic_name = topic_info->name;
    const std::string type_name = topic_info->type;

    auto open_decoder = core::decoder::open_decoder(*topic_info);
    if (!open_decoder.ok()) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to open decoder: %s", open_decoder.error.c_str());
      return 1;
    }
    const auto & decoder = *open_decoder.decoder;

    std::vector<OwnedMessage> cache;
    bool exhausted = false;

    auto load_next = [&]() -> bool {
      if (exhausted) {
        return false;
      }
      io::RawMessage raw;
      try {
        if (!reader->next(raw)) {
          exhausted = true;
          return false;
        }
      } catch (const std::exception & e) {
        BAGWIZ_LOG_ERROR(kLogger, "read error: %s", e.what());
        exhausted = true;
        return false;
      }
      cache.push_back({raw.timestamp_ns, copy_payload(raw.payload)});
      return true;
    };

    if (!load_next()) {
      fmt::print(
        stdout, "No messages found for topic '{}' in {}.\n", topic_name, input_path_.string());
      return 0;
    }

    // Image preview is offered only for topics to_packed_raster can decode and
    // only on terminals that speak a graphics protocol. Probe the terminal once,
    // before the pager owns stdin; the probe briefly enters raw mode so the
    // reply bytes arrive immediately instead of being held by line buffering.
    const bool is_image_topic = core::image::is_supported_image_type(type_name);
    core::tui::image::TerminalImageCaps image_caps;
    if (is_image_topic) {
      core::TerminalRawMode probe_raw;
      image_caps = core::tui::image::detect_terminal_image_caps(
        std::cout, STDIN_FILENO, core::tui::query_terminal_size());
    }
    const bool preview_available = is_image_topic && image_caps.can_render();

    std::size_t index = 0;
    bool expand_arrays = false;
    std::string status;

    core::tui::PagerConfig pager_cfg;
    core::tui::ScrollablePager pager(pager_cfg);

    // Append each wrapped fragment of `line` (wrapped at `cols`) onto
    // `out`. Continuation lines inherit the original's leading
    // whitespace via wrap_to_width.
    auto append_wrapped = [](std::vector<std::string> & out, std::string_view line, int cols) {
      auto wrapped = core::tui::wrap_to_width(line, cols);
      for (auto & w : wrapped) {
        out.push_back(std::move(w));
      }
    };

    auto build_frame = [&](std::size_t scroll, core::tui::Size term) -> core::tui::Frame {
      core::tui::Frame frame;

      const auto & msg = cache[index];
      const char * total_suffix = exhausted ? "" : "+";
      const std::size_t last_loaded_index = cache.size() - 1;

      // Header: build the two information rows, then wrap each one and
      // append a blank separator on its own logical line.
      const int cols = std::max(1, term.cols);
      append_wrapped(
        frame.header, fmt::format("timestamp: {}", format_timestamp(msg.timestamp_ns)), cols);
      append_wrapped(frame.header, fmt::format("size:      {} bytes", msg.payload.size()), cols);
      frame.header.emplace_back();  // blank separator

      core::FormatOptions fmt_opts;
      fmt_opts.expand_long_arrays = expand_arrays;
      const auto decoded = decoder.decode(msg.payload);
      const auto formatted = decoded.ok() ? core::format_message(*decoded.value, fmt_opts)
                                          : core::FormatResult{"", decoded.error};
      std::vector<std::string> body_logical;
      if (formatted.ok()) {
        body_logical = split_lines(formatted.text);
      } else {
        body_logical.push_back(
          fmt::format("⚠  Could not decode this message: {}", formatted.error));
      }
      frame.body.reserve(body_logical.size());
      for (const auto & line : body_logical) {
        append_wrapped(frame.body, line, cols);
      }

      // Footer: build logical lines first (blank separator, index row,
      // key legend, status row) so we know the wrapped footer height
      // before computing the scroll hint.
      std::vector<std::string> footer_logical;
      footer_logical.reserve(4);
      footer_logical.emplace_back();  // blank separator
      // The scroll hint depends on body_rows, which itself depends on
      // wrapped footer height. Resolve it iteratively below; emit a
      // placeholder index row for now and patch it once we know the
      // visible body window.
      footer_logical.emplace_back();
      std::string legend =
        "  [→ / Space] next   [← / b] prev   [,] -1s   [.] +1s   [↑ / k] up   [↓ / j] down   "
        "[Home / H] head   [End / T] tail   [g] first   [G] last   [s] save as yaml   "
        "[a] expand arrays   ";
      if (preview_available) {
        legend += rainbow_text("[i] preview");
        legend += "   ";
      }
      legend += "[q] quit";
      footer_logical.emplace_back(std::move(legend));
      footer_logical.push_back(status.empty() ? std::string{} : fmt::format("  {}", status));

      // Wrap everything except the index row first to learn the
      // footer's height; we know the index row will not change height
      // since it differs only in the trailing scroll hint, which we
      // append before re-wrapping.
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

      const std::string index_no_hint = fmt::format(
        "  [{} / {}{}]  {}  {}", index, last_loaded_index, total_suffix, topic_name, type_name);

      recompute_footer(index_no_hint);
      // Body rows available after the (current) footer wrap.
      auto body_rows_for = [&](const std::vector<std::string> & footer) {
        return std::max(
          0, term.rows - static_cast<int>(frame.header.size()) - static_cast<int>(footer.size()));
      };

      int body_rows = body_rows_for(footer_wrapped);
      const std::size_t total_body = frame.body.size();
      std::string scroll_hint;
      if (body_rows > 0 && total_body > static_cast<std::size_t>(body_rows)) {
        const std::size_t end = std::min(scroll + static_cast<std::size_t>(body_rows), total_body);
        scroll_hint = fmt::format("    lines {}-{} of {}", scroll + 1, end, total_body);
      }
      if (!scroll_hint.empty()) {
        recompute_footer(index_no_hint + scroll_hint);
        // Recomputing the footer can change its wrapped height (the
        // index row may now wrap), so re-derive body_rows once.
        body_rows = body_rows_for(footer_wrapped);
        if (total_body > static_cast<std::size_t>(std::max(body_rows, 0))) {
          const std::size_t end =
            std::min(scroll + static_cast<std::size_t>(body_rows), total_body);
          const std::string new_hint =
            fmt::format("    lines {}-{} of {}", scroll + 1, end, total_body);
          if (new_hint != scroll_hint) {
            scroll_hint = new_hint;
            recompute_footer(index_no_hint + scroll_hint);
          }
        }
      }

      frame.footer = std::move(footer_wrapped);
      return frame;
    };

    // Move the message cursor. Shared by the YAML view (on_nav) and the image
    // preview so both wrap, clamp at the first message, and full-scan on kLast
    // identically. Mutates index/cache/exhausted/status and returns whether the
    // cursor actually moved, so callers can skip work that only matters on a real
    // move (resetting the pager scroll offset; re-decoding the preview frame).
    auto navigate = [&](MsgNav move) -> bool {
      status.clear();
      const std::size_t before = index;
      switch (move) {
        case MsgNav::kNext:
          if (index + 1 < cache.size()) {
            ++index;
          } else if (load_next()) {
            index = cache.size() - 1;
          } else {
            index = 0;
            status = "(wrapped to first)";
          }
          break;
        case MsgNav::kPrev:
          if (index > 0) {
            --index;
          } else {
            status = "(at first message)";
          }
          break;
        case MsgNav::kFirst:
          index = 0;
          break;
        case MsgNav::kLast: {
          std::size_t loaded = 0;
          while (load_next()) {
            ++loaded;
          }
          index = cache.size() - 1;
          if (loaded == 0 && exhausted) {
            status = "(already at last message)";
          }
          break;
        }
        case MsgNav::kStepForward1s: {
          const int64_t target_ns = cache[index].timestamp_ns + kOneSecondNs;
          if (exhausted && cache.back().timestamp_ns < target_ns) {
            if (index == cache.size() - 1) {
              status = "(already at last message)";
            } else {
              index = cache.size() - 1;
              status = "(reached end)";
            }
            break;
          }
          while (!exhausted && cache.back().timestamp_ns < target_ns) {
            load_next();
          }
          auto it = std::lower_bound(
            cache.begin(), cache.end(), target_ns,
            [](const OwnedMessage & m, int64_t t) { return m.timestamp_ns < t; });
          if (it != cache.end()) {
            index = static_cast<std::size_t>(std::distance(cache.begin(), it));
          } else {
            index = cache.size() - 1;
            status = "(reached end)";
          }
          break;
        }
        case MsgNav::kStepBackward1s: {
          const int64_t target_ns = cache[index].timestamp_ns - kOneSecondNs;
          if (target_ns <= cache.front().timestamp_ns) {
            if (index == 0) {
              status = "(at first message)";
            } else {
              index = 0;
            }
            break;
          }
          auto it = std::upper_bound(
            cache.begin(), cache.end(), target_ns,
            [](int64_t t, const OwnedMessage & m) { return t < m.timestamp_ns; });
          // it is the first message strictly after target_ns; step back one
          // to land on the last message at or before target_ns.
          --it;
          index = static_cast<std::size_t>(std::distance(cache.begin(), it));
          break;
        }
      }
      return index != before;
    };

    auto on_nav = [&](core::tui::NavKey nav) -> core::tui::AppKeyResult {
      status.clear();
      switch (nav) {
        case core::tui::NavKey::kNext:
          navigate(MsgNav::kNext);
          pager.set_scroll_offset(0);
          return core::tui::AppKeyResult::kHandled;
        case core::tui::NavKey::kPrev:
          // Preserve the YAML view's behaviour: only reset the body scroll when
          // the cursor actually moved (pressing prev at the first message keeps
          // the current scroll position).
          if (navigate(MsgNav::kPrev)) {
            pager.set_scroll_offset(0);
          }
          return core::tui::AppKeyResult::kHandled;
        case core::tui::NavKey::kFirst:
          navigate(MsgNav::kFirst);
          pager.set_scroll_offset(0);
          return core::tui::AppKeyResult::kHandled;
        case core::tui::NavKey::kLast:
          navigate(MsgNav::kLast);
          pager.set_scroll_offset(0);
          return core::tui::AppKeyResult::kHandled;
        case core::tui::NavKey::kStepForward1s:
          navigate(MsgNav::kStepForward1s);
          pager.set_scroll_offset(0);
          return core::tui::AppKeyResult::kHandled;
        case core::tui::NavKey::kStepBackward1s:
          // Like prev, preserve body scroll when already at the boundary.
          if (navigate(MsgNav::kStepBackward1s)) {
            pager.set_scroll_offset(0);
          }
          return core::tui::AppKeyResult::kHandled;
        case core::tui::NavKey::kResize:
          return core::tui::AppKeyResult::kHandled;
        default:
          return core::tui::AppKeyResult::kIgnored;
      }
    };

    std::unique_ptr<core::image::UndistortHelper> undistort_helper;
    std::uint32_t undistort_helper_w = 0;
    std::uint32_t undistort_helper_h = 0;

    auto ensure_undistort_helper =
      [&](std::uint32_t w, std::uint32_t h) -> core::image::UndistortHelper * {
      if (!camera_info.has_value()) {
        return nullptr;
      }
      if (!undistort_helper || undistort_helper_w != w || undistort_helper_h != h) {
        undistort_helper = std::make_unique<core::image::UndistortHelper>(*camera_info, w, h);
        undistort_helper_w = w;
        undistort_helper_h = h;
      }
      return undistort_helper.get();
    };

    auto maybe_undistort = [&](core::image::PackedRaster * raster) {
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
    };

    bool undistort_enabled = false;

    // Paint one preview frame: a two-line caption, the decoded image centred in
    // the region between caption and key hint, and the key hint on the last row.
    // Graphics escapes bypass the pager (they have no display width), so this
    // writes straight to the pager's ostream.
    auto render_preview = [&](std::ostream & out, core::tui::Size term) {
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

      const auto & msg = cache[index];
      const char * total_suffix = exhausted ? "" : "+";
      const std::size_t last_loaded_index = cache.size() - 1;
      auto pr = core::image::to_packed_raster(type_name, msg.payload);
      if (undistort_enabled && pr.ok()) {
        maybe_undistort(&*pr.raster);
      }

      core::tui::draw_line(out, 1, fmt::format("  {}  {}", topic_name, type_name), cols);
      std::string info;
      if (pr.ok()) {
        const auto & img = *pr.raster;
        info = fmt::format(
          "  {}x{}  {}   [{} / {}{}]", img.width, img.height, img.encoding, index,
          last_loaded_index, total_suffix);
      } else {
        info = fmt::format("  [{} / {}{}]", index, last_loaded_index, total_suffix);
      }
      // Surface the save outcome (or any transient message) on the info row;
      // navigate() clears `status` on a cursor move, so it disappears as soon as
      // the user pages to another frame.
      if (!status.empty()) {
        info += fmt::format("   {}", status);
      }
      info += fmt::format("   undistort: {}", undistort_enabled ? "on" : "off");
      core::tui::draw_line(out, 2, info, cols);

      // Wrap the key legend the way the YAML footer (build_frame) does, so a
      // narrow terminal shows every key on continuation lines instead of
      // truncating the row. The wrapped legend is pinned to the bottom and the
      // image region above shrinks to make room, mirroring how build_frame
      // derives its body height from the wrapped footer.
      const std::vector<std::string> legend_lines = core::tui::wrap_to_width(
        "  [→ / Space] next   [← / b] prev   [,] -1s   [.] +1s   [g] first   [G] last   [s] save   "
        "[u] undistort   [i] back   [q] quit",
        cols);
      const int legend_top = std::max(1, rows - static_cast<int>(legend_lines.size()) + 1);

      // Image region: from row 3 down to the row above the first legend line.
      const int region_row = 3;
      const int region_rows = std::max(1, legend_top - region_row);
      if (pr.ok()) {
        core::tui::image::CellRegion region;
        region.row = region_row;
        region.col = 1;
        region.rows = region_rows;
        region.cols = cols;
        const std::string err = core::tui::image::render_image(out, *pr.raster, region, image_caps);
        if (!err.empty()) {
          core::tui::draw_line(
            out, region_row, fmt::format("  preview unavailable: {}", err), cols);
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
    };

    // Save the frame currently shown in the preview as a PNG. Mirrors the YAML
    // save handler (kSaveYaml): decode the message, prompt for a path via the
    // pager's cooked-mode line input, and write the bytes. The result is left in
    // `status`, which render_preview surfaces on the next repaint.
    auto save_preview_image = [&]() {
      status.clear();
      const auto & cur = cache[index];
      auto pr = core::image::to_packed_raster(type_name, cur.payload);
      if (!pr.ok()) {
        status = fmt::format("cannot save: {}", pr.error);
        return;
      }
      if (undistort_enabled) {
        maybe_undistort(&*pr.raster);
      }
      const auto encoded = core::image::encode_png(*pr.raster);
      if (!encoded.ok()) {
        status = fmt::format("cannot save: {}", encoded.error);
        return;
      }

      const std::string default_base =
        fmt::format("{}_{}.png", topic_for_filename(topic_name), index);
      std::filesystem::path cwd;
      try {
        cwd = std::filesystem::current_path();
      } catch (const std::exception & e) {
        status = fmt::format("cannot resolve working directory: {}", e.what());
        return;
      }
      const std::filesystem::path default_full = cwd / default_base;

      // Drop the on-screen graphic before switching to cooked-mode line input so
      // the prompt is not drawn over a kitty placement; run_preview repaints the
      // frame afterward.
      core::tui::image::clear_image(std::cout, image_caps.backend);
      std::cout << "\x1B[2J";
      std::cout.flush();

      std::filesystem::path out_path;
      bool save_ok = false;
      std::string failure_status;
      pager.with_line_input([&](std::istream & in, std::ostream & out) {
        out << fmt::format("Save image path (Enter for {}):\n", default_full.string());
        out.flush();
        std::string line;
        if (!std::getline(in, line)) {
          failure_status = "(save cancelled)";
          return;
        }
        out_path = resolve_save_path(line, cwd, default_base);
        std::error_code mk_ec;
        const auto parent = out_path.parent_path();
        if (!parent.empty()) {
          std::filesystem::create_directories(parent, mk_ec);
          if (mk_ec) {
            failure_status =
              fmt::format("could not create directory {}: {}", parent.string(), mk_ec.message());
            return;
          }
        }
        std::ofstream of(out_path, std::ios::binary);
        if (!of) {
          failure_status = fmt::format("could not open {} for writing", out_path.string());
          return;
        }
        const auto & bytes = *encoded.png;
        of.write(
          reinterpret_cast<const char *>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
            bytes.data()),
          static_cast<std::streamsize>(bytes.size()));
        if (!of.good()) {
          failure_status = fmt::format("write failed: {}", out_path.string());
          return;
        }
        save_ok = true;
      });
      if (save_ok) {
        status = fmt::format("saved {}", out_path.string());
      } else {
        status = failure_status;
      }
    };

    // Image-preview sub-loop. Runs inside on_app_key, reusing the raw-mode +
    // SIGWINCH scope the pager already holds. Navigation keys re-decode and
    // re-render; i/q return to the YAML view, which the pager then repaints.
    auto run_preview = [&]() {
      std::ostream & out = std::cout;
      bool running = true;
      bool needs_render = true;
      while (running) {
        if (needs_render) {
          render_preview(out, core::tui::query_terminal_size());
          needs_render = false;
        }
        switch (core::read_key_event()) {
          case core::KeyEvent::kNext:
            // Re-decode only when the cursor actually moved; otherwise the frame
            // is unchanged and a full decode + scale would be wasted.
            needs_render = navigate(MsgNav::kNext);
            break;
          case core::KeyEvent::kPrev:
            needs_render = navigate(MsgNav::kPrev);
            break;
          case core::KeyEvent::kFirst:
            needs_render = navigate(MsgNav::kFirst);
            break;
          case core::KeyEvent::kLast:
            needs_render = navigate(MsgNav::kLast);
            break;
          case core::KeyEvent::kStepForward1s:
            needs_render = navigate(MsgNav::kStepForward1s);
            break;
          case core::KeyEvent::kStepBackward1s:
            needs_render = navigate(MsgNav::kStepBackward1s);
            break;
          case core::KeyEvent::kResize:
            needs_render = true;  // geometry changed: re-fit and re-render
            break;
          case core::KeyEvent::kSaveYaml:
            // In the preview, [s] saves the displayed frame as a PNG (the YAML
            // view's [s] still saves YAML). Always repaint so the save status is
            // shown and the prompt's screen clear is undone.
            save_preview_image();
            needs_render = true;
            break;
          case core::KeyEvent::kToggleUndistort:
            if (!camera_info.has_value()) {
              status = camera_info_error.empty() ? "undistort: no camera_info"
                                                 : "undistort: " + camera_info_error;
            } else {
              undistort_enabled = !undistort_enabled;
            }
            needs_render = true;
            break;
          case core::KeyEvent::kTogglePreview:
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
    };

    auto on_app_key = [&](core::KeyEvent ev) -> core::tui::AppKeyResult {
      status.clear();
      switch (ev) {
        case core::KeyEvent::kToggleArrayExpand:
          expand_arrays = !expand_arrays;
          if (expand_arrays) {
            status = "(arrays: expanded)";
          }
          return core::tui::AppKeyResult::kHandled;
        case core::KeyEvent::kSaveYaml: {
          const auto & cur = cache[index];
          core::FormatOptions save_opts;
          save_opts.expand_long_arrays = expand_arrays;
          const auto decoded = decoder.decode(cur.payload);
          const auto formatted = decoded.ok() ? core::format_message(*decoded.value, save_opts)
                                              : core::FormatResult{"", decoded.error};
          if (!formatted.ok()) {
            status = fmt::format("cannot save: {}", formatted.error);
            return core::tui::AppKeyResult::kHandled;
          }
          const std::string default_base =
            fmt::format("{}_{}.yaml", topic_for_filename(topic_name), index);
          std::filesystem::path cwd;
          try {
            cwd = std::filesystem::current_path();
          } catch (const std::exception & e) {
            status = fmt::format("cannot resolve working directory: {}", e.what());
            return core::tui::AppKeyResult::kHandled;
          }
          const std::filesystem::path default_full = cwd / default_base;

          std::filesystem::path out_path;
          bool save_ok = false;
          std::string failure_status;
          pager.with_line_input([&](std::istream & in, std::ostream & out) {
            out << fmt::format("Save YAML path (Enter for {}):\n", default_full.string());
            out.flush();
            std::string line;
            if (!std::getline(in, line)) {
              failure_status = "(save cancelled)";
              return;
            }
            out_path = resolve_save_path(line, cwd, default_base);
            std::error_code mk_ec;
            const auto parent = out_path.parent_path();
            if (!parent.empty()) {
              std::filesystem::create_directories(parent, mk_ec);
              if (mk_ec) {
                failure_status = fmt::format(
                  "could not create directory {}: {}", parent.string(), mk_ec.message());
                return;
              }
            }
            std::ofstream of(out_path, std::ios::binary);
            if (!of) {
              failure_status = fmt::format("could not open {} for writing", out_path.string());
              return;
            }
            of << formatted.text;
            if (!of.good()) {
              failure_status = fmt::format("write failed: {}", out_path.string());
              return;
            }
            save_ok = true;
          });
          if (save_ok) {
            status = fmt::format("saved {}", out_path.string());
          } else {
            status = failure_status;
          }
          return core::tui::AppKeyResult::kHandled;
        }
        case core::KeyEvent::kTogglePreview:
          if (!is_image_topic) {
            status = "(not an image topic)";
            return core::tui::AppKeyResult::kHandled;
          }
          if (!image_caps.can_render()) {
            status = "(image preview not supported in this terminal)";
            return core::tui::AppKeyResult::kHandled;
          }
          run_preview();
          return core::tui::AppKeyResult::kHandled;
        default:
          return core::tui::AppKeyResult::kIgnored;
      }
    };

    return pager.run(build_frame, on_nav, on_app_key);
  }

private:
  std::filesystem::path input_path_;
  std::string topic_;
  std::optional<std::string> camera_info_topic_;
};

BAGWIZ_REGISTER_COMMAND(WalkCommand)

}  // namespace bagwiz::commands
