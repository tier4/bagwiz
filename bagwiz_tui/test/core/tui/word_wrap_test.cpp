// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tui/word_wrap.hpp"

#include "bagwiz/core/tui/width.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace
{

using bagwiz::core::tui::display_width;
using bagwiz::core::tui::word_wrap;

TEST(WordWrap, ShortTextStaysOnOneLine)
{
  const std::vector<std::string> lines = word_wrap("hello world", 80);
  ASSERT_EQ(lines.size(), 1U);
  EXPECT_EQ(lines[0], "hello world");
}

TEST(WordWrap, BreaksAtWordBoundary)
{
  // "aaa bbb" is exactly 7 columns and fits; adding " ccc" would overflow.
  const std::vector<std::string> lines = word_wrap("aaa bbb ccc", 7);
  ASSERT_EQ(lines.size(), 2U);
  EXPECT_EQ(lines[0], "aaa bbb");
  EXPECT_EQ(lines[1], "ccc");
}

TEST(WordWrap, NeverSplitsAWord)
{
  // Every produced line must respect the width except a lone over-long word.
  const std::vector<std::string> lines =
    word_wrap("the quick brown fox jumps over the lazy dog", 12);
  ASSERT_FALSE(lines.empty());
  for (const auto & line : lines) {
    EXPECT_LE(display_width(line), 12) << "line wider than max: " << line;
  }
}

TEST(WordWrap, OverlongWordOverflowsOnItsOwnLine)
{
  const std::vector<std::string> lines = word_wrap("supercalifragilistic", 5);
  ASSERT_EQ(lines.size(), 1U);
  EXPECT_EQ(lines[0], "supercalifragilistic");
}

TEST(WordWrap, OverlongWordDoesNotSwallowNeighbours)
{
  const std::vector<std::string> lines = word_wrap("hi supercalifragilistic ok", 5);
  ASSERT_EQ(lines.size(), 3U);
  EXPECT_EQ(lines[0], "hi");
  EXPECT_EQ(lines[1], "supercalifragilistic");
  EXPECT_EQ(lines[2], "ok");
}

TEST(WordWrap, PreservesHardNewlinesAsParagraphs)
{
  const std::vector<std::string> lines = word_wrap("line one\nline two", 80);
  ASSERT_EQ(lines.size(), 2U);
  EXPECT_EQ(lines[0], "line one");
  EXPECT_EQ(lines[1], "line two");
}

TEST(WordWrap, BlankParagraphSurvives)
{
  const std::vector<std::string> lines = word_wrap("a\n\nb", 80);
  ASSERT_EQ(lines.size(), 3U);
  EXPECT_EQ(lines[0], "a");
  EXPECT_EQ(lines[1], "");
  EXPECT_EQ(lines[2], "b");
}

TEST(WordWrap, CollapsesRunsOfWhitespace)
{
  const std::vector<std::string> lines = word_wrap("a   b\t c", 80);
  ASSERT_EQ(lines.size(), 1U);
  EXPECT_EQ(lines[0], "a b c");
}

TEST(WordWrap, TrimsLeadingAndTrailingWhitespace)
{
  const std::vector<std::string> lines = word_wrap("   hi there   ", 80);
  ASSERT_EQ(lines.size(), 1U);
  EXPECT_EQ(lines[0], "hi there");
}

TEST(WordWrap, EmptyInputYieldsOneEmptyLine)
{
  const std::vector<std::string> lines = word_wrap("", 80);
  ASSERT_EQ(lines.size(), 1U);
  EXPECT_EQ(lines[0], "");
}

TEST(WordWrap, WhitespaceOnlyInputYieldsOneEmptyLine)
{
  const std::vector<std::string> lines = word_wrap("    ", 80);
  ASSERT_EQ(lines.size(), 1U);
  EXPECT_EQ(lines[0], "");
}

TEST(WordWrap, NonPositiveWidthDisablesWrapping)
{
  const std::vector<std::string> lines = word_wrap("a b c d e f g", 0);
  ASSERT_EQ(lines.size(), 1U);
  EXPECT_EQ(lines[0], "a b c d e f g");
}

TEST(WordWrap, MeasuresCjkAsWidthTwo)
{
  // Three Hiragana "あ" (U+3042, 3 UTF-8 bytes, display width 2 each).
  const std::string a = "\xE3\x81\x82";
  const std::vector<std::string> lines = word_wrap(a + " " + a + " " + a, 5);
  // "あ あ" is 2 + 1 + 2 = 5 columns and fits; the third "あ" overflows.
  ASSERT_EQ(lines.size(), 2U);
  EXPECT_EQ(lines[0], a + " " + a);
  EXPECT_EQ(lines[1], a);
}

}  // namespace
