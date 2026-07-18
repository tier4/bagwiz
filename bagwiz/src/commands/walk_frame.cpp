// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "walk_frame.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include "bagwiz/core/base/str_utils.hpp"
#include "bagwiz/core/msg_yaml/message_formatter.hpp"
#include "bagwiz/core/tui/width.hpp"

#include <fmt/core.h>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{

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

void append_wrapped(std::vector<std::string> & out, std::string_view line, int cols)
{
  auto wrapped = core::tui::wrap_to_width(line, cols);
  for (auto & w : wrapped) {
    out.push_back(std::move(w));
  }
}

ScrollHintResolution resolve_scroll_hint(
  core::tui::Size term, std::size_t header_rows, const std::string & index_no_hint,
  std::vector<std::string> footer_logical, std::size_t total_body, std::size_t scroll)
{
  const int cols = std::max(1, term.cols);

  // Wrap everything except the index row first to learn the footer's height;
  // the index row does not change height when the scroll hint is appended
  // unless it wraps, which the second pass below accounts for.
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

  recompute_footer(index_no_hint);
  // Body rows available after the (current) footer wrap.
  auto body_rows_for = [&](const std::vector<std::string> & footer) {
    return std::max(0, term.rows - static_cast<int>(header_rows) - static_cast<int>(footer.size()));
  };

  int body_rows = body_rows_for(footer_wrapped);
  std::string scroll_hint;
  if (body_rows > 0 && total_body > static_cast<std::size_t>(body_rows)) {
    const std::size_t end = std::min(scroll + static_cast<std::size_t>(body_rows), total_body);
    scroll_hint = fmt::format("    lines {}-{} of {}", scroll + 1, end, total_body);
  }
  if (!scroll_hint.empty()) {
    recompute_footer(index_no_hint + scroll_hint);
    // Recomputing the footer can change its wrapped height (the index row
    // may now wrap), so re-derive body_rows once.
    body_rows = body_rows_for(footer_wrapped);
    if (total_body > static_cast<std::size_t>(std::max(body_rows, 0))) {
      const std::size_t end = std::min(scroll + static_cast<std::size_t>(body_rows), total_body);
      const std::string new_hint =
        fmt::format("    lines {}-{} of {}", scroll + 1, end, total_body);
      if (new_hint != scroll_hint) {
        scroll_hint = new_hint;
        recompute_footer(index_no_hint + scroll_hint);
      }
    }
  }

  ScrollHintResolution resolved;
  resolved.footer = std::move(footer_wrapped);
  resolved.scroll_hint = std::move(scroll_hint);
  resolved.body_rows = body_rows;
  return resolved;
}

core::tui::Frame build_yaml_frame(
  std::size_t scroll, core::tui::Size term, const MessageCursor & cursor,
  const core::decoder::Decoder & decoder, bool expand_arrays, const std::string & topic_name,
  const std::string & type_name, const std::string & status, bool preview_available)
{
  core::tui::Frame frame;

  const auto & msg = cursor.cache()[cursor.index()];
  const char * total_suffix = cursor.exhausted() ? "" : "+";
  const std::size_t last_loaded_index = cursor.cache().size() - 1;

  // Header: build the two information rows, then wrap each one and
  // append a blank separator on its own logical line.
  const int cols = std::max(1, term.cols);
  append_wrapped(
    frame.header, fmt::format("timestamp: {}", core::format_timestamp(msg.timestamp_ns)), cols);
  append_wrapped(frame.header, fmt::format("size:      {} bytes", msg.payload.size()), cols);
  frame.header.emplace_back();  // blank separator

  core::FormatOptions fmt_opts;
  fmt_opts.expand_long_arrays = expand_arrays;
  const auto decoded = decoder.decode(msg.payload);
  const auto formatted = decoded.ok() ? core::format_message(*decoded.value, fmt_opts)
                                      : core::FormatResult{"", decoded.error};
  std::vector<std::string> body_logical;
  if (formatted.ok()) {
    body_logical = core::split_lines(formatted.text);
  } else {
    body_logical.push_back(fmt::format("⚠  Could not decode this message: {}", formatted.error));
  }
  frame.body.reserve(body_logical.size());
  for (const auto & line : body_logical) {
    append_wrapped(frame.body, line, cols);
  }

  // Footer: build logical lines first (blank separator, index row, key
  // legend, status row) so the wrapped footer height is known before
  // computing the scroll hint.
  std::vector<std::string> footer_logical;
  footer_logical.reserve(4);
  footer_logical.emplace_back();  // blank separator
  // The scroll hint depends on body_rows, which itself depends on the
  // wrapped footer height. resolve_scroll_hint negotiates it iteratively;
  // emit a placeholder index row for it to patch.
  footer_logical.emplace_back();
  std::string legend =
    "  [→ / Space] next   [← / b] prev   [,] -1s   [.] +1s   [<] -10s   [>] +10s   [↑ / k] "
    "up   [↓ / j] down   "
    "[Home / H] head   [End / T] tail   [g] first   [G] last   [s] save as yaml   "
    "[a] expand arrays   ";
  if (preview_available) {
    legend += rainbow_text("[i] preview");
    legend += "   ";
  }
  legend += "[q] quit";
  footer_logical.emplace_back(std::move(legend));
  footer_logical.push_back(status.empty() ? std::string{} : fmt::format("  {}", status));

  const std::string index_no_hint = fmt::format(
    "  [{} / {}{}]  {}  {}", cursor.index(), last_loaded_index, total_suffix, topic_name,
    type_name);

  frame.footer = resolve_scroll_hint(
                   term, frame.header.size(), index_no_hint, std::move(footer_logical),
                   frame.body.size(), scroll)
                   .footer;
  return frame;
}

}  // namespace bagwiz::commands
