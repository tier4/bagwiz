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

// True when the Primary DA reply (`ESC [ ? Pn ; Pn ; ... c`) advertises Sixel,
// which is capability code 4. The parameter list is split on ';' and each token
// compared exactly to "4" so multi-digit codes like 14 or 40 never false-match.
// Allocation-free to stay noexcept-safe.
bool da1_advertises_sixel(std::string_view reply) noexcept
{
  const auto start = reply.find("\x1b[?");
  if (start == std::string_view::npos) {
    return false;
  }
  std::string_view params = reply.substr(start + 3);  // skip the "ESC [ ?" prefix
  if (const auto term = params.find('c'); term != std::string_view::npos) {
    params = params.substr(0, term);
  }
  std::size_t pos = 0;
  while (true) {
    const auto sep = params.find(';', pos);
    const auto end = sep == std::string_view::npos ? params.size() : sep;
    if (params.substr(pos, end - pos) == "4") {
      return true;
    }
    if (sep == std::string_view::npos) {
      return false;
    }
    pos = sep + 1;
  }
}
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
  // Kitty takes precedence; otherwise fall back to Sixel when the Primary DA
  // reply advertises capability 4. Selection order: kitty -> sixel -> none.
  if (da1_advertises_sixel(reply)) {
    return ImageBackend::kSixel;
  }
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
    // kitty query produced has already arrived ahead of it. Search for the `c`
    // from the `ESC [ ?` onward so a stray `c` earlier in the stream cannot end
    // the read before the full DA1 parameter list (which carries the Sixel
    // capability) has arrived.
    if (const auto da1 = reply.find("\x1b[?");
        da1 != std::string::npos && reply.find('c', da1) != std::string::npos) {
      break;
    }
  }

  caps.backend = classify_query_reply(reply);
  return caps;
}

}  // namespace bagwiz::core::tui::image
