// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tui/image/terminal_image_caps.hpp"

#include "bagwiz/core/tui/layout.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <array>
#include <cstddef>
#include <sstream>
#include <string>
#include <string_view>

namespace
{
using bagwiz::core::tui::Size;
using bagwiz::core::tui::image::cell_pixels;
using bagwiz::core::tui::image::classify_query_reply;
using bagwiz::core::tui::image::detect_terminal_image_caps;
using bagwiz::core::tui::image::ImageBackend;

// A Kitty-capable terminal answers the graphics query with an APC `;OK`, then
// the DA1 reply; a non-Kitty terminal answers only DA1.
constexpr std::string_view kKittyOk = "\x1b_Gi=31;OK\x1b\\\x1b[?62;1;c";
constexpr std::string_view kDa1Only = "\x1b[?62;1;c";

// Feed `reply` to the probe through a pipe (closed after the canned bytes so the
// read sees EOF) and return the detected backend.
ImageBackend probe_with_reply(std::string_view reply, Size term)
{
  std::array<int, 2> fds{-1, -1};
  if (::pipe(fds.data()) != 0) {
    ADD_FAILURE() << "pipe() failed";
    return ImageBackend::kNone;
  }
  if (!reply.empty()) {
    const ssize_t w = ::write(fds[1], reply.data(), reply.size());
    EXPECT_EQ(static_cast<std::size_t>(w), reply.size());
  }
  ::close(fds[1]);  // EOF after the canned reply
  std::ostringstream out;
  const auto caps = detect_terminal_image_caps(out, fds[0], term);
  ::close(fds[0]);
  // The probe must have emitted the DA1 query.
  EXPECT_NE(out.str().find("\x1b[c"), std::string::npos);
  return caps.backend;
}

Size term_80x24()
{
  Size t;
  t.cols = 80;
  t.rows = 24;
  return t;
}

// --- classify_query_reply (pure) ---------------------------------------------

TEST(TerminalImageCapsTest, ClassifyKittyOk)
{
  EXPECT_EQ(classify_query_reply(kKittyOk), ImageBackend::kKitty);
}

TEST(TerminalImageCapsTest, ClassifyDa1OnlyIsNone)
{
  EXPECT_EQ(classify_query_reply(kDa1Only), ImageBackend::kNone);
}

TEST(TerminalImageCapsTest, ClassifyEmptyIsNone)
{
  EXPECT_EQ(classify_query_reply(""), ImageBackend::kNone);
}

TEST(TerminalImageCapsTest, ClassifyGarbageIsNone)
{
  EXPECT_EQ(classify_query_reply("random \x1b[0m noise"), ImageBackend::kNone);
}

// --- cell_pixels -------------------------------------------------------------

TEST(TerminalImageCapsTest, CellPixelsFromReportedDims)
{
  Size t = term_80x24();
  t.xpixel = 800;
  t.ypixel = 480;
  const auto c = cell_pixels(t);
  EXPECT_EQ(c.width, 10);
  EXPECT_EQ(c.height, 20);
}

TEST(TerminalImageCapsTest, CellPixelsFallbackWhenUnreported)
{
  const auto c = cell_pixels(term_80x24());  // xpixel/ypixel default 0
  EXPECT_GT(c.width, 0);
  EXPECT_GT(c.height, 0);
  EXPECT_GE(c.height, c.width);  // assumed ~1:2 cell
}

TEST(TerminalImageCapsTest, CellPixelsClampsToPositive)
{
  Size t = term_80x24();
  t.xpixel = 40;  // smaller than cols -> would divide to 0
  t.ypixel = 12;  // smaller than rows
  const auto c = cell_pixels(t);
  EXPECT_GE(c.width, 1);
  EXPECT_GE(c.height, 1);
}

// --- detect_terminal_image_caps (via a pipe) ---------------------------------

TEST(TerminalImageCapsTest, DetectKittyViaPipe)
{
  EXPECT_EQ(probe_with_reply(kKittyOk, term_80x24()), ImageBackend::kKitty);
}

TEST(TerminalImageCapsTest, DetectNoneOnDa1Only)
{
  EXPECT_EQ(probe_with_reply(kDa1Only, term_80x24()), ImageBackend::kNone);
}

TEST(TerminalImageCapsTest, DetectNoneOnEmptyReply)
{
  EXPECT_EQ(probe_with_reply("", term_80x24()), ImageBackend::kNone);
}

}  // namespace
