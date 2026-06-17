// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tui/layout.hpp"

#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>

namespace bagwiz::core::tui
{

namespace
{
constexpr int kFallbackRows = 24;
constexpr int kFallbackCols = 80;
}  // namespace

Size query_terminal_size() noexcept
{
  Size out;
  out.rows = kFallbackRows;
  out.cols = kFallbackCols;
  struct winsize ws{};
  if (::ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws) == 0 && ws.ws_row > 0 && ws.ws_col > 0) {
    out.rows = static_cast<int>(ws.ws_row);
    out.cols = static_cast<int>(ws.ws_col);
    out.xpixel = static_cast<int>(ws.ws_xpixel);  // may be 0 if unreported
    out.ypixel = static_cast<int>(ws.ws_ypixel);  // may be 0 if unreported
  }
  return out;
}

int FixedLayout::body_end_row() const noexcept
{
  return size.rows - footer_rows;
}

int FixedLayout::body_rows() const noexcept
{
  return std::max(1, body_end_row() - body_start_row() + 1);
}

int FixedLayout::footer_start_row() const noexcept
{
  return size.rows - footer_rows + 1;
}

}  // namespace bagwiz::core::tui
