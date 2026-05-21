// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tui/width.hpp"

#include <gtest/gtest.h>

namespace
{

using bagwiz::core::tui::display_width;
using bagwiz::core::tui::truncate_to_width;
using bagwiz::core::tui::wrap_to_width;

TEST(DisplayWidth, EmptyIsZero)
{
  EXPECT_EQ(display_width(""), 0);
}

TEST(DisplayWidth, AsciiLetters)
{
  EXPECT_EQ(display_width("hello"), 5);
}

TEST(DisplayWidth, AsciiPunctuation)
{
  EXPECT_EQ(display_width("{}[]"), 4);
}

TEST(DisplayWidth, SgrEscapesAreZeroWidth)
{
  EXPECT_EQ(display_width("\x1B[31mred\x1B[0m"), 3);
}

TEST(DisplayWidth, CsiCursorEscapesAreZeroWidth)
{
  EXPECT_EQ(display_width("\x1B[5;1H"), 0);
}

TEST(DisplayWidth, ControlCharsZeroWidth)
{
  EXPECT_EQ(display_width("\t\r"), 0);
  EXPECT_EQ(display_width("\x01\x07"), 0);
}

TEST(DisplayWidth, NonAsciiLatinCountsAsOne)
{
  // "café" - 'é' is a single non-CJK codepoint -> width 1
  EXPECT_EQ(display_width("caf\xC3\xA9"), 4);
}

TEST(DisplayWidth, CjkHiraganaIsWidthTwo)
{
  // "あ" U+3042 (Hiragana A) -> 3 bytes UTF-8 "\xE3\x81\x82" -> width 2
  EXPECT_EQ(display_width("\xE3\x81\x82"), 2);
}

TEST(DisplayWidth, CjkKanjiIsWidthTwo)
{
  // "日本" - 2 kanji characters, 6 bytes UTF-8 -> width 4
  EXPECT_EQ(display_width("\xE6\x97\xA5\xE6\x9C\xAC"), 4);
}

TEST(DisplayWidth, HangulSyllableIsWidthTwo)
{
  // "한" U+D55C -> width 2
  EXPECT_EQ(display_width("\xED\x95\x9C"), 2);
}

TEST(DisplayWidth, MixedAsciiCjk)
{
  // "a日b" -> 1 + 2 + 1 = 4
  EXPECT_EQ(
    display_width(
      "a\xE6\x97\xA5"
      "b"),
    4);
}

TEST(DisplayWidth, IncompleteUtf8IsIgnored)
{
  // A bare 0xE3 with no continuation bytes -> width 0 (no codepoint
  // consumed, never half-counted).
  EXPECT_EQ(display_width("\xE3"), 0);
}

TEST(DisplayWidth, EscapeWithoutBracketIsWidthOne)
{
  // A lone ESC byte that is not a CSI prefix is not zero-width — but
  // also not a complete escape. Treat it as width 0 (control char).
  EXPECT_EQ(display_width("\x1B"), 0);
}

TEST(TruncateToWidth, NegativeReturnsEmpty)
{
  EXPECT_EQ(truncate_to_width("abc", -1), "");
}

TEST(TruncateToWidth, ZeroReturnsEmpty)
{
  EXPECT_EQ(truncate_to_width("abc", 0), "");
}

TEST(TruncateToWidth, FitsUnchanged)
{
  EXPECT_EQ(truncate_to_width("abc", 5), "abc");
  EXPECT_EQ(truncate_to_width("abc", 3), "abc");
}

TEST(TruncateToWidth, CutsAtBoundary)
{
  EXPECT_EQ(truncate_to_width("abcdef", 3), "abc");
}

TEST(TruncateToWidth, NeverSplitsCodepoint)
{
  // "caf\xC3\xA9def" widths 1,1,1,1,1,1,1 = 7; cut at 4 should give "café"
  // (i.e., includes the full 'é' codepoint), 4 bytes is the boundary
  // after 'caf' (3 bytes) but cut should not split the 0xC3 0xA9 pair.
  EXPECT_EQ(
    truncate_to_width(
      "caf\xC3\xA9"
      "def",
      4),
    "caf\xC3\xA9");
  EXPECT_EQ(
    truncate_to_width(
      "caf\xC3\xA9"
      "def",
      3),
    "caf");
}

TEST(TruncateToWidth, WideCharNotIncludedIfWouldOverflow)
{
  // "a日" width = 1 + 2 = 3. With max_cols = 2, including 日 would push
  // to width 3, so it must be dropped, leaving "a".
  EXPECT_EQ(truncate_to_width("a\xE6\x97\xA5", 2), "a");
  EXPECT_EQ(truncate_to_width("a\xE6\x97\xA5", 3), "a\xE6\x97\xA5");
}

TEST(TruncateToWidth, PreservesSgrPrefix)
{
  // "\x1B[31m" is width 0 and atomic. Width budget of 3 should keep the
  // SGR prefix plus "abc".
  EXPECT_EQ(
    truncate_to_width(
      "\x1B[31m"
      "abcdef",
      3),
    "\x1B[31m"
    "abc");
}

TEST(TruncateToWidth, DropsTrailingPartialEscape)
{
  // "abc\x1B[3" - the trailing CSI is incomplete; truncate should drop
  // it. With max_cols = 5, "abc" fits (width 3), and the partial CSI
  // must not be emitted as garbled escape bytes.
  EXPECT_EQ(truncate_to_width("abc\x1B[3", 5), "abc");
}

TEST(TruncateToWidth, IncludesCompleteCsiThenStops)
{
  // Two complete CSIs followed by content past the budget.
  EXPECT_EQ(
    truncate_to_width(
      "\x1B[31m\x1B[1m"
      "abcdef",
      2),
    "\x1B[31m\x1B[1m"
    "ab");
}

TEST(WrapToWidth, EmptyReturnsSingleBlankLine)
{
  const auto wrapped = wrap_to_width("", 10);
  ASSERT_EQ(wrapped.size(), 1U);
  EXPECT_EQ(wrapped[0], "");
}

TEST(WrapToWidth, NegativeMaxColsReturnsInputUnchanged)
{
  const auto wrapped = wrap_to_width("abcdef", -1);
  ASSERT_EQ(wrapped.size(), 1U);
  EXPECT_EQ(wrapped[0], "abcdef");
}

TEST(WrapToWidth, FitsInOneLine)
{
  const auto wrapped = wrap_to_width("hello", 10);
  ASSERT_EQ(wrapped.size(), 1U);
  EXPECT_EQ(wrapped[0], "hello");
}

TEST(WrapToWidth, HardWrapAsciiNoIndent)
{
  const auto wrapped = wrap_to_width("abcdefgh", 3);
  ASSERT_EQ(wrapped.size(), 3U);
  EXPECT_EQ(wrapped[0], "abc");
  EXPECT_EQ(wrapped[1], "def");
  EXPECT_EQ(wrapped[2], "gh");
}

TEST(WrapToWidth, ContinuationInheritsLeadingSpaces)
{
  // 4-space indent + 6 content chars at max_cols=8 -> first line fits
  // "    abcd" (8 cols), continuation line carries the same indent.
  const auto wrapped = wrap_to_width("    abcdef", 8);
  ASSERT_EQ(wrapped.size(), 2U);
  EXPECT_EQ(wrapped[0], "    abcd");
  EXPECT_EQ(wrapped[1], "    ef");
}

TEST(WrapToWidth, IndentDroppedWhenItFillsTheLine)
{
  // Indent (4 spaces) == max_cols=4 -> a continuation indented to that
  // width would have no room for content, so drop it on continuations.
  const auto wrapped = wrap_to_width("    abcdef", 4);
  // First segment may briefly tolerate the indent because it is the
  // original prefix, but continuation lines must not carry it.
  ASSERT_GE(wrapped.size(), 2U);
  EXPECT_EQ(wrapped.back().find_first_not_of(' '), 0U);
}

TEST(WrapToWidth, CjkWideCodepointNeverSplit)
{
  // "ab日本" widths 1+1+2+2 = 6. max_cols=4 must keep "日" intact:
  // first line "ab日" (width 4), continuation "本" (width 2).
  const auto wrapped = wrap_to_width("ab\xE6\x97\xA5\xE6\x9C\xAC", 4);
  ASSERT_EQ(wrapped.size(), 2U);
  EXPECT_EQ(wrapped[0], "ab\xE6\x97\xA5");
  EXPECT_EQ(wrapped[1], "\xE6\x9C\xAC");
}

TEST(WrapToWidth, CsiEscapesAttachWithoutAffectingWrap)
{
  // CSI sequences are zero-width and atomic.
  const auto wrapped = wrap_to_width("\x1B[31mabcdef\x1B[0m", 3);
  ASSERT_EQ(wrapped.size(), 2U);
  EXPECT_EQ(wrapped[0], "\x1B[31mabc");
  EXPECT_EQ(wrapped[1], "def\x1B[0m");
}

TEST(WrapToWidth, BlankWhitespaceOnlyLineIsPreserved)
{
  // A whitespace-only line is shorter than max_cols and round-trips as-is.
  const auto wrapped = wrap_to_width("   ", 10);
  ASSERT_EQ(wrapped.size(), 1U);
  EXPECT_EQ(wrapped[0], "   ");
}

}  // namespace
