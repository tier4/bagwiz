// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tui/image/terminal_image_renderer.hpp"

#include "bagwiz/core/image/packed_raster.hpp"
#include "bagwiz/core/tui/image/terminal_image_caps.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <span>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace
{
using bagwiz::core::image::PackedRaster;
using bagwiz::core::tui::image::base64_encode;
using bagwiz::core::tui::image::CellPixels;
using bagwiz::core::tui::image::CellRegion;
using bagwiz::core::tui::image::clear_image;
using bagwiz::core::tui::image::fit_image;
using bagwiz::core::tui::image::ImageBackend;
using bagwiz::core::tui::image::ImageFit;
using bagwiz::core::tui::image::render_image;
using bagwiz::core::tui::image::TerminalImageCaps;

std::vector<std::byte> bytes_of(std::string_view s)
{
  std::vector<std::byte> out(s.size());
  for (std::size_t i = 0; i < s.size(); ++i) {
    out[i] = static_cast<std::byte>(static_cast<unsigned char>(s[i]));
  }
  return out;
}

std::string b64(std::string_view s)
{
  const auto buf = bytes_of(s);
  return base64_encode(std::span<const std::byte>(buf.data(), buf.size()));
}

// Build a w x h canonical BGR24 raster with deterministic, valid pixel bytes.
PackedRaster make_raster(std::uint32_t w, std::uint32_t h)
{
  PackedRaster r;
  r.width = w;
  r.height = h;
  r.encoding = "bgr8";
  r.bgr.resize(static_cast<std::size_t>(w) * 3U * static_cast<std::size_t>(h));
  for (std::size_t i = 0; i < r.bgr.size(); ++i) {
    r.bgr[i] = static_cast<std::byte>(static_cast<unsigned char>(i & 0xFFU));
  }
  return r;
}

TerminalImageCaps kitty_caps(int cell_w, int cell_h)
{
  TerminalImageCaps caps;
  caps.backend = ImageBackend::kKitty;
  caps.cell = CellPixels{cell_w, cell_h};
  return caps;
}

TerminalImageCaps sixel_caps(int cell_w, int cell_h)
{
  TerminalImageCaps caps;
  caps.backend = ImageBackend::kSixel;
  caps.cell = CellPixels{cell_w, cell_h};
  return caps;
}

// --- base64_encode (RFC 4648 test vectors) -----------------------------------

TEST(TerminalImageRendererTest, Base64EmptyIsEmpty)
{
  EXPECT_EQ(b64(""), "");
}

// cspell:ignore foob fooba
TEST(TerminalImageRendererTest, Base64Rfc4648Vectors)
{
  EXPECT_EQ(b64("f"), "Zg==");
  EXPECT_EQ(b64("fo"), "Zm8=");
  EXPECT_EQ(b64("foo"), "Zm9v");
  EXPECT_EQ(b64("foob"), "Zm9vYg==");
  EXPECT_EQ(b64("fooba"), "Zm9vYmE=");
  EXPECT_EQ(b64("foobar"), "Zm9vYmFy");
}

// --- fit_image (pure geometry) -----------------------------------------------

TEST(TerminalImageRendererTest, FitSquareImageCenters)
{
  const ImageFit fit = fit_image(100, 100, CellRegion{2, 1, 20, 40}, CellPixels{10, 20});
  EXPECT_EQ(fit.px_width, 360);
  EXPECT_EQ(fit.px_height, 360);
  EXPECT_EQ(fit.cells_wide, 36);
  EXPECT_EQ(fit.cells_high, 18);
  EXPECT_EQ(fit.row, 3);
  EXPECT_EQ(fit.col, 3);
}

TEST(TerminalImageRendererTest, FitWideImageIsHeightPadded)
{
  const ImageFit fit = fit_image(200, 100, CellRegion{2, 1, 20, 40}, CellPixels{10, 20});
  EXPECT_EQ(fit.px_width, 360);
  EXPECT_EQ(fit.px_height, 180);
  EXPECT_EQ(fit.cells_wide, 36);
  EXPECT_EQ(fit.cells_high, 9);
  EXPECT_EQ(fit.row, 7);  // (20 - 9) / 2 = 5 rows of top padding
  EXPECT_EQ(fit.col, 3);
}

TEST(TerminalImageRendererTest, FitDegenerateInputYieldsZeroPixels)
{
  const ImageFit fit = fit_image(0, 0, CellRegion{1, 1, 10, 10}, CellPixels{10, 20});
  EXPECT_EQ(fit.px_width, 0);
  EXPECT_EQ(fit.px_height, 0);
  EXPECT_EQ(fit.row, 1);
  EXPECT_EQ(fit.col, 1);
}

// --- render_image (Kitty framing) --------------------------------------------

TEST(TerminalImageRendererTest, RenderKittyEmitsSingleChunkFraming)
{
  // A tiny cell + region forces a tiny scaled image whose base64 fits one chunk.
  const PackedRaster raster = make_raster(8, 8);
  const CellRegion region{1, 1, 3, 3};
  const TerminalImageCaps caps = kitty_caps(2, 2);
  const ImageFit fit = fit_image(raster.width, raster.height, region, caps.cell);

  std::ostringstream out;
  const std::string err = render_image(out, raster, region, caps);
  ASSERT_EQ(err, "");
  const std::string s = out.str();

  // Cursor is positioned at the centered top-left cell before the graphics.
  const std::string cup = "\x1b[" + std::to_string(fit.row) + ";" + std::to_string(fit.col) + "H";
  EXPECT_NE(s.find(cup), std::string::npos);
  // First (and only) chunk carries the control keys and m=0 (no more chunks).
  const std::string header = "\x1b_Gf=24,s=" + std::to_string(fit.px_width) +
                             ",v=" + std::to_string(fit.px_height) + ",a=T,m=0;";
  EXPECT_NE(s.find(header), std::string::npos);
  // Escape terminator present.
  ASSERT_GE(s.size(), 2U);
  EXPECT_EQ(s.substr(s.size() - 2), "\x1b\\");
}

TEST(TerminalImageRendererTest, RenderKittyChunksLargePayload)
{
  // A large fit produces > 4096 base64 bytes, so the payload must be split:
  // every chunk but the last carries m=1, the last carries m=0.
  const PackedRaster raster = make_raster(8, 8);
  const CellRegion region{3, 1, 10, 40};
  const TerminalImageCaps caps = kitty_caps(10, 20);

  std::ostringstream out;
  const std::string err = render_image(out, raster, region, caps);
  ASSERT_EQ(err, "");
  const std::string s = out.str();

  // Control keys ride only the first chunk, which is a continuation (m=1).
  EXPECT_NE(s.find(",a=T,m=1;"), std::string::npos);
  EXPECT_NE(s.find("m=0;"), std::string::npos);
}

TEST(TerminalImageRendererTest, RenderSixelEmitsDcsFraming)
{
  // Exercises the real libsixel encoder; assert the DCS framing rather than
  // exact bytes (palette/quantization is libsixel-version dependent).
  const PackedRaster raster = make_raster(8, 8);
  const CellRegion region{1, 1, 6, 12};
  const TerminalImageCaps caps = sixel_caps(6, 12);
  const ImageFit fit = fit_image(raster.width, raster.height, region, caps.cell);

  std::ostringstream out;
  const std::string err = render_image(out, raster, region, caps);
  ASSERT_EQ(err, "");
  const std::string s = out.str();

  // Cursor positioned at the centered top-left cell, then a Sixel DCS string.
  const std::string cup = "\x1b[" + std::to_string(fit.row) + ";" + std::to_string(fit.col) + "H";
  EXPECT_NE(s.find(cup), std::string::npos);
  EXPECT_NE(s.find("\x1bP"), std::string::npos);   // DCS introducer (ESC P)
  EXPECT_NE(s.find("\x1b\\"), std::string::npos);  // ST terminator (ESC \)
}

TEST(TerminalImageRendererTest, RenderEmptyRasterErrorsWithoutOutput)
{
  PackedRaster empty;  // width == height == 0
  std::ostringstream out;
  const std::string err = render_image(out, empty, CellRegion{1, 1, 10, 10}, kitty_caps(10, 20));
  EXPECT_FALSE(err.empty());
  EXPECT_TRUE(out.str().empty());
}

TEST(TerminalImageRendererTest, RenderNoneBackendErrorsWithoutOutput)
{
  const PackedRaster raster = make_raster(8, 8);
  TerminalImageCaps caps;  // backend defaults to kNone
  std::ostringstream out;
  const std::string err = render_image(out, raster, CellRegion{1, 1, 10, 10}, caps);
  EXPECT_FALSE(err.empty());
  EXPECT_TRUE(out.str().empty());
}

// --- clear_image -------------------------------------------------------------

TEST(TerminalImageRendererTest, ClearKittyDeletesAll)
{
  std::ostringstream out;
  clear_image(out, ImageBackend::kKitty);
  EXPECT_EQ(out.str(), "\x1b_Ga=d,d=A\x1b\\");
}

TEST(TerminalImageRendererTest, ClearNoneIsNoOp)
{
  std::ostringstream out;
  clear_image(out, ImageBackend::kNone);
  EXPECT_TRUE(out.str().empty());
}

TEST(TerminalImageRendererTest, ClearSixelIsNoOp)
{
  std::ostringstream out;
  clear_image(out, ImageBackend::kSixel);
  EXPECT_TRUE(out.str().empty());
}

}  // namespace
