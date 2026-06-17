// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tui/image/terminal_image_renderer.hpp"

#include "bagwiz/core/tui/renderer.hpp"

extern "C" {
#include <libavutil/imgutils.h>
#include <libavutil/mem.h>
#include <libavutil/pixfmt.h>
#include <libswscale/swscale.h>
}

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <ostream>
#include <span>
#include <string>
#include <vector>

namespace bagwiz::core::tui::image
{
namespace
{
using bagwiz::core::image::PackedRaster;

// Kitty recommends at most 4096 bytes of base64 payload per transmit escape.
constexpr std::size_t kKittyChunkBytes = 4096;

// RAII for the libswscale resize handles, so every early return frees them
// (mirrors image_decoder.cpp's DecodeContext).
struct ScaleContext
{
  SwsContext * sws = nullptr;
  std::uint8_t * dst_buf = nullptr;  // av_image_alloc'd swscale destination

  ScaleContext() = default;
  ScaleContext(const ScaleContext &) = delete;
  ScaleContext & operator=(const ScaleContext &) = delete;
  ScaleContext(ScaleContext &&) = delete;
  ScaleContext & operator=(ScaleContext &&) = delete;

  ~ScaleContext()
  {
    if (dst_buf != nullptr) {
      av_freep(&dst_buf);
    }
    if (sws != nullptr) {
      sws_freeContext(sws);
    }
  }
};

// Resize the canonical BGR24 `raster` to `dst_w` x `dst_h` and convert to packed
// RGB24 (the order Kitty's f=24 transmission expects). On success fills `out_rgb`
// with dst_w * 3 * dst_h bytes and returns ""; otherwise returns the reason.
std::string scale_to_rgb24(
  const PackedRaster & raster, int dst_w, int dst_h, std::vector<std::byte> & out_rgb)
{
  // Guard the int casts and the `* 3` stride below against overflow. Real sensor
  // images are nowhere near this; a pathological width/height is rejected rather
  // than wrapping to a negative linesize.
  constexpr std::uint32_t kMaxDim = static_cast<std::uint32_t>(std::numeric_limits<int>::max() / 3);
  if (raster.width > kMaxDim || raster.height > kMaxDim) {
    return "image dimensions are too large for the preview";
  }
  const int src_w = static_cast<int>(raster.width);
  const int src_h = static_cast<int>(raster.height);
  if (src_w <= 0 || src_h <= 0) {
    return "image has invalid dimensions";
  }
  if (dst_w <= 0 || dst_h <= 0) {
    return "preview target size is empty";
  }
  const std::size_t need = static_cast<std::size_t>(src_w) * 3U * static_cast<std::size_t>(src_h);
  if (raster.bgr.size() < need) {
    return "image buffer is smaller than its dimensions";
  }

  ScaleContext ctx;
  ctx.sws = sws_getContext(
    src_w, src_h, AV_PIX_FMT_BGR24, dst_w, dst_h, AV_PIX_FMT_RGB24, SWS_BILINEAR, nullptr, nullptr,
    nullptr);
  if (ctx.sws == nullptr) {
    return "failed to create swscale context for the preview";
  }

  std::array<const std::uint8_t *, 4> src_data{};
  std::array<int, 4> src_linesize{};
  src_data[0] =
    reinterpret_cast<const std::uint8_t *>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
      raster.bgr.data());
  src_linesize[0] = src_w * 3;

  // sws_scale's SIMD paths can write past a tightly-packed row, so give it an
  // aligned, padded destination via av_image_alloc; copy the packed rows out
  // afterward (same discipline as image_decoder.cpp).
  std::array<std::uint8_t *, 4> dst_data{};
  std::array<int, 4> dst_linesize{};
  const int alloc =
    av_image_alloc(dst_data.data(), dst_linesize.data(), dst_w, dst_h, AV_PIX_FMT_RGB24, 16);
  if (alloc < 0) {
    return "could not allocate preview output buffer";
  }
  ctx.dst_buf = dst_data[0];  // owned by ScaleContext; freed on any return below

  const int scaled = sws_scale(
    ctx.sws, src_data.data(), src_linesize.data(), 0, src_h, dst_data.data(), dst_linesize.data());
  if (scaled != dst_h) {
    return "swscale produced an unexpected number of rows";
  }

  const int row_bytes = dst_w * 3;
  out_rgb.assign(
    static_cast<std::size_t>(row_bytes) * static_cast<std::size_t>(dst_h), std::byte{0});
  for (int y = 0; y < dst_h; ++y) {
    std::memcpy(
      out_rgb.data() + static_cast<std::ptrdiff_t>(y) * row_bytes,
      ctx.dst_buf + static_cast<std::ptrdiff_t>(y) * dst_linesize[0],
      static_cast<std::size_t>(row_bytes));
  }
  return "";
}

// Emit a Kitty graphics transmit-and-display (a=T) escape for `rgb` (packed
// RGB24, w*h*3 bytes), positioned at 1-based (row, col). Control keys ride only
// the first chunk; the base64 payload is split into <=kKittyChunkBytes pieces
// with m=1 on every chunk but the last.
void emit_kitty(std::ostream & out, std::span<const std::byte> rgb, int w, int h, int row, int col)
{
  if (rgb.empty()) {
    return;  // nothing to transmit; never emit a degenerate s=0,v=0 escape
  }
  move_cursor(out, row, col);

  const std::string payload = base64_encode(rgb);
  const std::size_t total = payload.size();
  std::size_t pos = 0;
  bool first = true;
  while (first || pos < total) {
    const std::size_t take = std::min(kKittyChunkBytes, total - pos);
    const bool last = pos + take >= total;
    out << "\x1b_G";
    if (first) {
      out << "f=24,s=" << w << ",v=" << h << ",a=T,";
    }
    out << "m=" << (last ? '0' : '1') << ';';
    out.write(payload.data() + pos, static_cast<std::streamsize>(take));
    out << "\x1b\\";
    pos += take;
    first = false;
    if (last) {
      break;
    }
  }
}

}  // namespace

ImageFit fit_image(
  std::uint32_t img_width, std::uint32_t img_height, CellRegion region, CellPixels cell) noexcept
{
  ImageFit fit;
  fit.row = region.row;
  fit.col = region.col;
  if (
    img_width == 0 || img_height == 0 || region.rows <= 0 || region.cols <= 0 || cell.width <= 0 ||
    cell.height <= 0) {
    return fit;  // degenerate input -> zero-pixel fit at the region origin
  }

  // Usable cell box (region shrunk by the fill factor, at least one cell).
  const int box_cells_w = std::max(1, static_cast<int>(region.cols * kPreviewFillFactor));
  const int box_cells_h = std::max(1, static_cast<int>(region.rows * kPreviewFillFactor));
  const double box_px_w = static_cast<double>(box_cells_w) * cell.width;
  const double box_px_h = static_cast<double>(box_cells_h) * cell.height;

  // Aspect-preserved fit into the pixel box (may upscale small images).
  const double scale = std::min(box_px_w / img_width, box_px_h / img_height);
  fit.px_width = std::max(1, static_cast<int>(std::lround(img_width * scale)));
  fit.px_height = std::max(1, static_cast<int>(std::lround(img_height * scale)));

  // Whole-cell footprint (ceil), clamped to the region so centering never goes
  // negative even if rounding nudged the footprint past the box.
  fit.cells_wide = std::min(region.cols, std::max(1, (fit.px_width + cell.width - 1) / cell.width));
  fit.cells_high =
    std::min(region.rows, std::max(1, (fit.px_height + cell.height - 1) / cell.height));

  fit.row = region.row + (region.rows - fit.cells_high) / 2;
  fit.col = region.col + (region.cols - fit.cells_wide) / 2;
  return fit;
}

std::string base64_encode(std::span<const std::byte> data)
{
  static constexpr char kTable[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve(((data.size() + 2) / 3) * 4);

  const std::size_t n = data.size();
  auto byte_at = [&](std::size_t k) {
    return static_cast<unsigned>(std::to_integer<unsigned char>(data[k]));
  };

  std::size_t i = 0;
  for (; i + 3 <= n; i += 3) {
    const unsigned triple = (byte_at(i) << 16) | (byte_at(i + 1) << 8) | byte_at(i + 2);
    out.push_back(kTable[(triple >> 18) & 0x3FU]);
    out.push_back(kTable[(triple >> 12) & 0x3FU]);
    out.push_back(kTable[(triple >> 6) & 0x3FU]);
    out.push_back(kTable[triple & 0x3FU]);
  }
  if (const std::size_t rem = n - i; rem == 1) {
    const unsigned triple = byte_at(i) << 16;
    out.push_back(kTable[(triple >> 18) & 0x3FU]);
    out.push_back(kTable[(triple >> 12) & 0x3FU]);
    out.push_back('=');
    out.push_back('=');
  } else if (rem == 2) {
    const unsigned triple = (byte_at(i) << 16) | (byte_at(i + 1) << 8);
    out.push_back(kTable[(triple >> 18) & 0x3FU]);
    out.push_back(kTable[(triple >> 12) & 0x3FU]);
    out.push_back(kTable[(triple >> 6) & 0x3FU]);
    out.push_back('=');
  }
  return out;
}

std::string render_image(
  std::ostream & out, const PackedRaster & raster, CellRegion region,
  const TerminalImageCaps & caps)
{
  if (raster.empty()) {
    return "image is empty";
  }
  if (!caps.can_render()) {
    return "terminal has no supported graphics backend";
  }
  if (region.rows <= 0 || region.cols <= 0) {
    return "preview region is empty";
  }

  const ImageFit fit = fit_image(raster.width, raster.height, region, caps.cell);
  if (fit.px_width <= 0 || fit.px_height <= 0) {
    return "preview region is too small for the image";
  }

  std::vector<std::byte> rgb;
  if (std::string err = scale_to_rgb24(raster, fit.px_width, fit.px_height, rgb); !err.empty()) {
    return err;
  }

  switch (caps.backend) {
    case ImageBackend::kKitty:
      emit_kitty(out, rgb, fit.px_width, fit.px_height, fit.row, fit.col);
      return "";
    case ImageBackend::kSixel:
      return "sixel image preview is not available yet";  // PR 3
    case ImageBackend::kNone:
    default:
      return "terminal has no supported graphics backend";
  }
}

void clear_image(std::ostream & out, ImageBackend backend)
{
  if (backend == ImageBackend::kKitty) {
    // a=d with d=A deletes all transmitted images and their placements, so no
    // stale frame survives behind the repainted YAML view.
    out << "\x1b_Ga=d,d=A\x1b\\";
  }
  // kSixel / kNone: the caller's screen clear is sufficient.
}

}  // namespace bagwiz::core::tui::image
