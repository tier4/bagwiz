// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__WALK_FRAME_HPP_
#define COMMANDS__WALK_FRAME_HPP_

#include "bagwiz/core/decoder/decoder.hpp"
#include "bagwiz/core/tui/layout.hpp"
#include "bagwiz/core/tui/pager.hpp"
#include "walk_cursor.hpp"  // NOLINT(build/include_subdir) src-local shared header

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

// YAML pager-frame construction of `bagwiz walk`: the header/body/footer
// assembly and the iterative footer/scroll-hint negotiation. Pure rendering
// (no terminal I/O), so it is unit-testable without a TTY. CLI-internal:
// this header lives with the command sources and is not installed.
namespace bagwiz::commands
{

// Append each wrapped fragment of `line` (wrapped at `cols`) onto `out`.
// Continuation lines inherit the original's leading whitespace via
// wrap_to_width.
void append_wrapped(std::vector<std::string> & out, std::string_view line, int cols);

// Outcome of resolve_scroll_hint(): the final wrapped footer (index row
// patched), the resolved scroll hint (empty when the body fits without
// scrolling), and the body rows visible between header and footer.
struct ScrollHintResolution
{
  std::vector<std::string> footer;
  std::string scroll_hint;
  int body_rows = 0;
};

// Resolve the footer for walk's YAML view. `footer_logical` carries the
// footer's logical lines with a placeholder at index 1, which this function
// overwrites with the index row: first without a hint to learn the wrapped
// footer height, then — when the body overflows — again with the hint
// appended, re-deriving the body window once because the longer index row
// may itself wrap and shrink the window further.
[[nodiscard]] ScrollHintResolution resolve_scroll_hint(
  core::tui::Size term, std::size_t header_rows, const std::string & index_no_hint,
  std::vector<std::string> footer_logical, std::size_t total_body, std::size_t scroll);

// Build one pager frame for the YAML view at `scroll`: the two information
// header rows (timestamp, size), the decoded message body, and the pinned
// footer (index row with scroll hint, key legend, status row). The legend
// gains a rainbow "[i] preview" hint when `preview_available`.
[[nodiscard]] core::tui::Frame build_yaml_frame(
  std::size_t scroll, core::tui::Size term, const MessageCursor & cursor,
  const core::decoder::Decoder & decoder, bool expand_arrays, const std::string & topic_name,
  const std::string & type_name, const std::string & status, bool preview_available);

}  // namespace bagwiz::commands

#endif  // COMMANDS__WALK_FRAME_HPP_
