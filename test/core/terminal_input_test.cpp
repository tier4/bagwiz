// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/terminal_input.hpp"

#include <gtest/gtest.h>

#include <string_view>

namespace
{

using bagwiz::core::classify_key;
using bagwiz::core::KeyEvent;

TEST(ClassifyKey, EmptyIsUnknown)
{
  EXPECT_EQ(classify_key(""), KeyEvent::kUnknown);
}

TEST(ClassifyKey, NextBindings)
{
  EXPECT_EQ(classify_key(" "), KeyEvent::kNext);
  EXPECT_EQ(classify_key(std::string_view("\x1B[C", 3)), KeyEvent::kNext);
}

TEST(ClassifyKey, PrevBindings)
{
  EXPECT_EQ(classify_key("b"), KeyEvent::kPrev);
  EXPECT_EQ(classify_key(std::string_view("\x1B[D", 3)), KeyEvent::kPrev);
}

TEST(ClassifyKey, PreviouslyRetiredKeysAreUnknown)
{
  // These used to map to next/prev; they were dropped when the key set
  // was narrowed to arrows + space + b. Pin the current contract so a
  // future re-binding is an intentional edit.
  EXPECT_EQ(classify_key("l"), KeyEvent::kUnknown);
  EXPECT_EQ(classify_key("n"), KeyEvent::kUnknown);
  EXPECT_EQ(classify_key("h"), KeyEvent::kUnknown);
  EXPECT_EQ(classify_key("p"), KeyEvent::kUnknown);
}

TEST(ClassifyKey, FirstAndLast)
{
  EXPECT_EQ(classify_key("g"), KeyEvent::kFirst);
  EXPECT_EQ(classify_key("G"), KeyEvent::kLast);
}

TEST(ClassifyKey, QuitBindings)
{
  EXPECT_EQ(classify_key("q"), KeyEvent::kQuit);
  EXPECT_EQ(classify_key("Q"), KeyEvent::kQuit);
  EXPECT_EQ(classify_key(std::string_view("\x1B", 1)), KeyEvent::kQuit);  // lone ESC
  EXPECT_EQ(classify_key(std::string_view("\x03", 1)), KeyEvent::kQuit);  // Ctrl-C
  EXPECT_EQ(classify_key(std::string_view("\x04", 1)), KeyEvent::kQuit);  // Ctrl-D
}

TEST(ClassifyKey, SaveYamlBinding)
{
  EXPECT_EQ(classify_key("s"), KeyEvent::kSaveYaml);
}

TEST(ClassifyKey, ToggleArrayExpandBinding)
{
  EXPECT_EQ(classify_key("a"), KeyEvent::kToggleArrayExpand);
}

TEST(ClassifyKey, ToggleRotationBinding)
{
  EXPECT_EQ(classify_key("r"), KeyEvent::kToggleRotation);
}

TEST(ClassifyKey, ScrollBindings)
{
  EXPECT_EQ(classify_key("k"), KeyEvent::kScrollUp);
  EXPECT_EQ(classify_key(std::string_view("\x1B[A", 3)), KeyEvent::kScrollUp);  // Up arrow
  EXPECT_EQ(classify_key("j"), KeyEvent::kScrollDown);
  EXPECT_EQ(classify_key(std::string_view("\x1B[B", 3)), KeyEvent::kScrollDown);  // Down arrow
  EXPECT_EQ(classify_key("H"), KeyEvent::kScrollHead);
  EXPECT_EQ(classify_key(std::string_view("\x1B[H", 3)), KeyEvent::kScrollHead);  // Home
  EXPECT_EQ(classify_key("T"), KeyEvent::kScrollTail);
  EXPECT_EQ(classify_key(std::string_view("\x1B[F", 3)), KeyEvent::kScrollTail);  // End
}

TEST(ClassifyKey, UnknownSequences)
{
  EXPECT_EQ(classify_key("x"), KeyEvent::kUnknown);
  EXPECT_EQ(classify_key("\t"), KeyEvent::kUnknown);
  // CSI with an unmapped final character.
  EXPECT_EQ(classify_key(std::string_view("\x1B[E", 3)), KeyEvent::kUnknown);
  // Two-byte (partial ESC [) -> unknown; callers handle this by prefetching.
  EXPECT_EQ(classify_key(std::string_view("\x1B[", 2)), KeyEvent::kUnknown);
}

TEST(ClassifyKey, ResizeIsNeverProducedByClassify)
{
  // kResize is synthesised by read_key_event() from a SIGWINCH flag,
  // never returned from byte classification. Pin this so a future
  // refactor that conflates the two paths is caught.
  for (int b = 0; b < 256; ++b) {
    const auto ch = static_cast<char>(b);
    EXPECT_NE(classify_key(std::string_view(&ch, 1)), KeyEvent::kResize);
  }
}

}  // namespace
