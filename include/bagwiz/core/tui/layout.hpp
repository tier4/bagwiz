// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__TUI__LAYOUT_HPP_
#define BAGWIZ__CORE__TUI__LAYOUT_HPP_

namespace bagwiz::core::tui
{

// Terminal geometry. rows/cols are character cells; xpixel/ypixel are the
// cell-grid size in pixels as reported by TIOCGWINSZ (ws_xpixel/ws_ypixel),
// or 0 when the terminal does not report them (common — many xterms, tmux).
struct Size
{
  int rows = 24;
  int cols = 80;
  int xpixel = 0;  // total grid width in pixels; 0 = unknown
  int ypixel = 0;  // total grid height in pixels; 0 = unknown
};

// Query the current terminal size via TIOCGWINSZ on STDOUT_FILENO.
// Returns a sane fallback (24x80) when stdout is not a TTY or the
// ioctl fails. Never throws.
Size query_terminal_size() noexcept;

// FixedLayout computes the absolute row ranges (1-based, inclusive) of
// the header, body, and footer regions for a given terminal size and
// fixed header/footer line counts.
//
// All row accessors return 1-based row indices suitable for direct use
// with the CUP escape `\x1B[<row>;<col>H`. Body collapses to a single
// row when the header + footer demand exceeds the terminal height.
struct FixedLayout
{
  Size size;
  int header_rows = 0;
  int footer_rows = 0;

  // NOLINTNEXTLINE(readability-convert-member-functions-to-static)
  [[nodiscard]] int header_start_row() const noexcept { return 1; }
  [[nodiscard]] int header_end_row() const noexcept { return header_rows; }
  [[nodiscard]] int body_start_row() const noexcept { return header_rows + 1; }
  [[nodiscard]] int body_end_row() const noexcept;
  [[nodiscard]] int body_rows() const noexcept;
  [[nodiscard]] int footer_start_row() const noexcept;
  [[nodiscard]] int footer_end_row() const noexcept { return size.rows; }
};

}  // namespace bagwiz::core::tui

#endif  // BAGWIZ__CORE__TUI__LAYOUT_HPP_
