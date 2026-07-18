// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__TUI__IMAGE__TERMINAL_IMAGE_CAPS_HPP_
#define BAGWIZ__CORE__TUI__IMAGE__TERMINAL_IMAGE_CAPS_HPP_

#include "bagwiz/core/tui/layout.hpp"

#include <ostream>
#include <string_view>

// Detects which terminal graphics protocol (if any) the current terminal
// supports, so `walk` can offer an image preview only where it can actually
// render. This is protocol-only: Kitty graphics and DEC Sixel. There is
// deliberately no half-block / truecolor fallback — a terminal that supports
// neither protocol reports kNone and the preview is simply unavailable.
namespace bagwiz::core::tui::image
{

enum class ImageBackend {
  kNone,   // no supported graphics protocol; preview unavailable
  kSixel,  // DEC Sixel (libsixel)
  kKitty,  // Kitty graphics protocol
};

// Pixel size of one character cell. Always strictly positive.
struct CellPixels
{
  int width = 0;
  int height = 0;
};

struct TerminalImageCaps
{
  ImageBackend backend = ImageBackend::kNone;
  CellPixels cell;

  [[nodiscard]] bool can_render() const noexcept { return backend != ImageBackend::kNone; }
};

// Per-cell pixel size derived from `term`. When the terminal reports pixel
// dimensions (xpixel/ypixel > 0) the cell size is xpixel/cols by ypixel/rows;
// otherwise it falls back to an assumed ~1:2 cell. Never returns a zero
// dimension.
[[nodiscard]] CellPixels cell_pixels(Size term) noexcept;

// Classify an accumulated query-reply byte string into a backend. Pure; exposed
// so the parser can be tested without real terminal I/O.
[[nodiscard]] ImageBackend classify_query_reply(std::string_view reply) noexcept;

// Probe the terminal exactly once, BEFORE the pager takes over stdin. Writes the
// graphics-capability query escapes to `out`, then reads replies from `in_fd`
// with a bounded timeout and fully drains them so no reply bytes leak into later
// key reads. `term` supplies the cell geometry. A terminal that answers neither
// query (or is not a TTY) yields kNone. Does not throw under normal operation.
[[nodiscard]] TerminalImageCaps detect_terminal_image_caps(
  std::ostream & out, int in_fd, Size term);

}  // namespace bagwiz::core::tui::image

#endif  // BAGWIZ__CORE__TUI__IMAGE__TERMINAL_IMAGE_CAPS_HPP_
