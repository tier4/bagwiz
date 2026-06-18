// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tui/renderer.hpp"

#include "bagwiz/core/tui/width.hpp"

#include <ostream>
#include <string>
#include <string_view>

namespace bagwiz::core::tui
{

void move_cursor(std::ostream & out, int row, int col)
{
  // Hand-roll the CUP sequence to avoid pulling fmt's consteval format
  // machinery into a header that is also exercised by clang-tidy (the
  // tidy run trips over fmt's compile-time checks).
  out << "\x1B[" << row << ';' << col << 'H';
}

void erase_in_line(std::ostream & out)
{
  out << "\x1B[2K";
}

void hide_cursor(std::ostream & out)
{
  out << "\x1B[?25l";
}

void show_cursor(std::ostream & out)
{
  out << "\x1B[?25h";
}

void set_autowrap(std::ostream & out, bool on)
{
  out << (on ? "\x1B[?7h" : "\x1B[?7l");
}

void enter_alt_screen(std::ostream & out)
{
  out << "\x1B[?1049h";
}

void leave_alt_screen(std::ostream & out)
{
  out << "\x1B[?1049l";
}

void begin_synchronized_update(std::ostream & out)
{
  out << "\x1B[?2026h";
}

void end_synchronized_update(std::ostream & out)
{
  out << "\x1B[?2026l";
}

void draw_line(std::ostream & out, int row, std::string_view text, int max_cols)
{
  move_cursor(out, row, 1);
  erase_in_line(out);
  if (max_cols <= 0 || text.empty()) {
    return;
  }
  const std::string clipped = truncate_to_width(text, max_cols);
  out.write(clipped.data(), static_cast<std::streamsize>(clipped.size()));
}

}  // namespace bagwiz::core::tui
