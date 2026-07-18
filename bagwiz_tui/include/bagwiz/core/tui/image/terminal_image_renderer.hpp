// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__TUI__IMAGE__TERMINAL_IMAGE_RENDERER_HPP_
#define BAGWIZ__CORE__TUI__IMAGE__TERMINAL_IMAGE_RENDERER_HPP_

#include "bagwiz/core/image/packed_raster.hpp"
#include "bagwiz/core/tui/image/terminal_image_caps.hpp"

#include <cstdint>
#include <ostream>
#include <span>
#include <string>

// Renders a decoded image into a rectangular cell region of the terminal using
// the detected graphics protocol. Protocol-only — Kitty graphics here, Sixel in
// PR 3; a terminal with neither never reaches this code (its caps report kNone).
// libswscale (resize + BGR->RGB) stays inside the .cpp; this header exposes only
// std types. The pure geometry/base64 helpers are exported so the transmit
// framing can be tested without a live terminal.
namespace bagwiz::core::tui::image
{

// 1-based rectangle of character cells the preview may draw into.
struct CellRegion
{
  int row = 1;
  int col = 1;
  int rows = 0;
  int cols = 0;
};

// Fraction of the region the image is allowed to occupy before centering; the
// leftover is fixed padding so the image never touches the region edges.
inline constexpr double kPreviewFillFactor = 0.9;

// Result of fitting an image into a region: the pixel size transmitted to the
// terminal, its whole-cell footprint, and the 1-based top-left cell that centers
// that footprint inside the region.
struct ImageFit
{
  int px_width = 0;
  int px_height = 0;
  int cells_wide = 0;
  int cells_high = 0;
  int row = 1;
  int col = 1;
};

// Aspect-preserved fit of an `img_width` x `img_height` image into
// `region * kPreviewFillFactor`, centered, using per-cell pixel size `cell`.
// Images smaller than that box are upscaled to fill it (a deliberate design
// choice — kPreviewFillFactor is the only padding knob), so the returned pixel
// size may exceed the source. Pure, deterministic integer math; never divides by
// zero (a zero-size image or region yields a zero-pixel fit at the region origin).
[[nodiscard]] ImageFit fit_image(
  std::uint32_t img_width, std::uint32_t img_height, CellRegion region, CellPixels cell) noexcept;

// Standard RFC 4648 base64 with `=` padding. Pure; exposed so the Kitty transmit
// framing can be asserted byte-exact in tests without a real terminal.
[[nodiscard]] std::string base64_encode(std::span<const std::byte> data);

// Render `raster` into `region` via `caps.backend`, writing graphics escapes to
// `out`. The image is scaled aspect-preserved into `region * kPreviewFillFactor`
// (BGR->RGB happens in the scale step) and centered: the cursor is moved to the
// centered top-left cell before the escapes are emitted. Kitty transmits the
// frame as an APC graphics command; Sixel encodes it via libsixel. Returns "" on
// success, or a human-readable reason on failure (empty raster, kNone backend,
// scale failure, sixel encode failure). Does not clear the region or reposition
// the cursor afterward — the caller composes the caption/hint rows around it.
// Never throws.
[[nodiscard]] std::string render_image(
  std::ostream & out, const bagwiz::core::image::PackedRaster & raster, CellRegion region,
  const TerminalImageCaps & caps);

// Tell the terminal to forget any graphics this renderer transmitted (Kitty
// delete-all). No-op for backends where the caller's screen clear already
// suffices (Sixel) or where nothing was drawn (kNone).
void clear_image(std::ostream & out, ImageBackend backend);

}  // namespace bagwiz::core::tui::image

#endif  // BAGWIZ__CORE__TUI__IMAGE__TERMINAL_IMAGE_RENDERER_HPP_
