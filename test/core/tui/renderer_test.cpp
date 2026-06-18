// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tui/renderer.hpp"

#include <gtest/gtest.h>

#include <sstream>
#include <string>

namespace
{

namespace tui = bagwiz::core::tui;

TEST(Renderer, MoveCursorEmitsCsi)
{
  std::ostringstream os;
  tui::move_cursor(os, 5, 1);
  EXPECT_EQ(os.str(), "\x1B[5;1H");
}

TEST(Renderer, MoveCursorOtherCoords)
{
  std::ostringstream os;
  tui::move_cursor(os, 23, 80);
  EXPECT_EQ(os.str(), "\x1B[23;80H");
}

TEST(Renderer, EraseInLine)
{
  std::ostringstream os;
  tui::erase_in_line(os);
  EXPECT_EQ(os.str(), "\x1B[2K");
}

TEST(Renderer, HideAndShowCursor)
{
  std::ostringstream os;
  tui::hide_cursor(os);
  tui::show_cursor(os);
  EXPECT_EQ(os.str(), "\x1B[?25l\x1B[?25h");
}

TEST(Renderer, SetAutowrapOnOff)
{
  std::ostringstream os_on;
  tui::set_autowrap(os_on, true);
  EXPECT_EQ(os_on.str(), "\x1B[?7h");

  std::ostringstream os_off;
  tui::set_autowrap(os_off, false);
  EXPECT_EQ(os_off.str(), "\x1B[?7l");
}

TEST(Renderer, EnterLeaveAltScreen)
{
  std::ostringstream os;
  tui::enter_alt_screen(os);
  tui::leave_alt_screen(os);
  EXPECT_EQ(os.str(), "\x1B[?1049h\x1B[?1049l");
}

TEST(Renderer, SynchronizedUpdateBracketsEmitMode2026)
{
  std::ostringstream os;
  tui::begin_synchronized_update(os);
  tui::end_synchronized_update(os);
  EXPECT_EQ(os.str(), "\x1B[?2026h\x1B[?2026l");
}

TEST(Renderer, DrawLineEmitsMoveEraseText)
{
  std::ostringstream os;
  tui::draw_line(os, 3, "hello", 80);
  // Adjacent string literals keep the control prefix separate from the visible
  // text so the spell checker does not read "\x1B[2K" + "hello" as one token.
  EXPECT_EQ(
    os.str(),
    "\x1B[3;1H\x1B[2K"
    "hello");
}

TEST(Renderer, DrawLineTruncatesToColumns)
{
  std::ostringstream os;
  tui::draw_line(os, 1, "hello world", 5);
  // After move + erase, the visible text is exactly 5 columns wide. The control
  // prefix and visible text are separate literals (see the note above).
  EXPECT_EQ(
    os.str(),
    "\x1B[1;1H\x1B[2K"
    "hello");
}

TEST(Renderer, DrawLineNeverEmitsNewline)
{
  std::ostringstream os;
  tui::draw_line(os, 1, "anything", 80);
  EXPECT_EQ(os.str().find('\n'), std::string::npos);
}

TEST(Renderer, DrawLineEmptyText)
{
  std::ostringstream os;
  tui::draw_line(os, 1, "", 80);
  // Still moves and erases, just with no text appended.
  EXPECT_EQ(os.str(), "\x1B[1;1H\x1B[2K");
}

TEST(Renderer, DrawLineZeroCols)
{
  std::ostringstream os;
  tui::draw_line(os, 1, "ignored", 0);
  EXPECT_EQ(os.str(), "\x1B[1;1H\x1B[2K");
}

}  // namespace
