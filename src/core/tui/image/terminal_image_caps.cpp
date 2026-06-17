// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tui/image/terminal_image_caps.hpp"

#include <poll.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <string>
#include <string_view>

namespace bagwiz::core::tui::image
{

namespace
{
// Assumed cell aspect (~1:2 width:height) when the terminal does not report
// pixel dimensions. Only the ratio matters for fit; the absolute values are a
// reasonable monospace default.
constexpr int kAssumedCellWidth = 10;
constexpr int kAssumedCellHeight = 20;

constexpr int kProbeTimeoutMs = 100;
constexpr int kPollSliceMs = 20;
constexpr std::size_t kMaxReplyBytes = 4096;

// Kitty graphics capability query (a=q) for a 1x1 RGB cell, followed by Primary
// DA. A Kitty-capable terminal answers the first with an APC `ESC _G ... ;OK
// ESC \`; every terminal answers DA1 with `ESC [ ? ... c`, which we use as a
// reliable read terminator even when the kitty query is ignored.
constexpr std::string_view kKittyQuery = "\x1b_Gi=31,s=1,v=1,a=q,t=d,f=24;AAAA\x1b\\";
constexpr std::string_view kDa1Query = "\x1b[c";
}  // namespace

CellPixels cell_pixels(Size term) noexcept
{
  CellPixels cell{kAssumedCellWidth, kAssumedCellHeight};
  if (term.xpixel > 0 && term.cols > 0) {
    cell.width = std::max(1, term.xpixel / term.cols);
  }
  if (term.ypixel > 0 && term.rows > 0) {
    cell.height = std::max(1, term.ypixel / term.rows);
  }
  return cell;
}

ImageBackend classify_query_reply(std::string_view reply) noexcept
{
  // Kitty answers the graphics query with an APC `ESC _G ... ESC \`. Require the
  // ";OK" status to fall *inside* that APC frame so a stray ";OK" elsewhere in
  // the reply stream (e.g. a terminal's DA1 extension) cannot be misread as a
  // Kitty success.
  if (const auto start = reply.find("\x1b_G"); start != std::string_view::npos) {
    std::string_view frame = reply.substr(start);
    if (const auto term = frame.find("\x1b\\"); term != std::string_view::npos) {
      frame = frame.substr(0, term);
    }
    if (frame.find(";OK") != std::string_view::npos) {
      return ImageBackend::kKitty;
    }
  }
  // PR 3 will additionally classify Sixel from the DA1 reply's `;4` capability.
  return ImageBackend::kNone;
}

TerminalImageCaps detect_terminal_image_caps(std::ostream & out, int in_fd, Size term)
{
  TerminalImageCaps caps;
  caps.cell = cell_pixels(term);

  out << kKittyQuery << kDa1Query;
  out.flush();

  std::string reply;
  int waited_ms = 0;
  while (waited_ms < kProbeTimeoutMs && reply.size() < kMaxReplyBytes) {
    struct ::pollfd pfd{};
    pfd.fd = in_fd;
    pfd.events = POLLIN;
    const int rc = ::poll(&pfd, 1, kPollSliceMs);
    if (rc < 0) {
      break;  // poll error (e.g. EINTR) — give up, report kNone
    }
    if (rc == 0) {
      waited_ms += kPollSliceMs;
      continue;  // no reply yet
    }
    if ((pfd.revents & POLLIN) == 0) {
      break;  // POLLERR/POLLHUP/POLLNVAL
    }
    std::array<char, 256> buf{};
    const ssize_t n = ::read(in_fd, buf.data(), buf.size());
    if (n <= 0) {
      break;  // EOF or read error
    }
    reply.append(buf.data(), static_cast<std::size_t>(n));
    // DA1 reply (`ESC [ ? ... c`) is the terminator; once present, whatever the
    // kitty query produced has already arrived ahead of it.
    if (reply.find("\x1b[?") != std::string::npos && reply.find('c') != std::string::npos) {
      break;
    }
  }

  caps.backend = classify_query_reply(reply);
  return caps;
}

}  // namespace bagwiz::core::tui::image
