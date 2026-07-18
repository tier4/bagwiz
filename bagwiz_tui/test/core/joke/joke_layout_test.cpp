// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/joke/joke_layout.hpp"

#include "bagwiz/core/tui/width.hpp"

#include <gtest/gtest.h>

#include <cctype>
#include <sstream>
#include <string>
#include <vector>

namespace
{

// A comfortably wide terminal, the common case.
constexpr int kWideTerminal = 80;

constexpr const char * kLongJoke =
  "This is a deliberately long joke that has to be split across several "
  "separate lines so that it stays comfortably readable beside the face.";

std::vector<std::string> split_lines(const std::string & text)
{
  std::vector<std::string> lines;
  std::istringstream stream(text);
  std::string line;
  while (std::getline(stream, line)) {
    lines.push_back(line);
  }
  return lines;
}

bool has_ascii_letter(const std::string & line)
{
  for (const unsigned char c : line) {
    if (std::isalpha(c) != 0) {
      return true;
    }
  }
  return false;
}

}  // namespace

TEST(JokeLayoutTest, RendersTheWojak)
{
  const auto out = bagwiz::core::joke::render_joke("short joke", kWideTerminal);
  // The block-character Wojak is present and there is no speech bubble.
  EXPECT_NE(out.find("▄"), std::string::npos);
  EXPECT_EQ(out.find("╭"), std::string::npos);
  EXPECT_EQ(out.find("│"), std::string::npos);
}

TEST(JokeLayoutTest, ShowsTheJokeText)
{
  const auto out = bagwiz::core::joke::render_joke("hello world", kWideTerminal);
  EXPECT_NE(out.find("hello world"), std::string::npos);
}

TEST(JokeLayoutTest, PlacesTextToTheRightOfTheFace)
{
  const auto out = bagwiz::core::joke::render_joke("rightside", kWideTerminal);

  bool found = false;
  for (const auto & line : split_lines(out)) {
    const auto pos = line.find("rightside");
    if (pos != std::string::npos) {
      found = true;
      // Text is indented to the right of the face, never flush-left.
      EXPECT_GT(pos, 0U) << "joke text should not start at column 0: " << line;
    }
  }
  EXPECT_TRUE(found);
}

TEST(JokeLayoutTest, WrapsLongJokeIntoSeveralLines)
{
  const auto out = bagwiz::core::joke::render_joke(kLongJoke, kWideTerminal);

  int text_lines = 0;
  for (const auto & line : split_lines(out)) {
    if (has_ascii_letter(line)) {
      ++text_lines;
    }
  }
  EXPECT_GT(text_lines, 1);
}

// The core fix: on a narrow terminal the text must wrap within its own
// column so that no rendered line is wider than the terminal — otherwise the
// terminal hard-wraps it back to column 0, under the face.
TEST(JokeLayoutTest, NarrowTerminalKeepsEveryLineWithinWidth)
{
  constexpr int kNarrow = 60;
  const auto out = bagwiz::core::joke::render_joke(kLongJoke, kNarrow);

  for (const auto & line : split_lines(out)) {
    EXPECT_LE(bagwiz::core::tui::display_width(line), kNarrow)
      << "line wider than the terminal would hard-wrap under the face: " << line;
  }
}

// Even on a very wide terminal the text keeps a bounded measure rather than
// becoming one long line.
TEST(JokeLayoutTest, WideTerminalStillWrapsToAReadableMeasure)
{
  const auto out = bagwiz::core::joke::render_joke(kLongJoke, 200);

  int text_lines = 0;
  for (const auto & line : split_lines(out)) {
    if (has_ascii_letter(line)) {
      ++text_lines;
    }
  }
  EXPECT_GT(text_lines, 1);
}

TEST(JokeLayoutTest, EndsWithANewline)
{
  const auto out = bagwiz::core::joke::render_joke("trailing newline check", kWideTerminal);
  ASSERT_FALSE(out.empty());
  EXPECT_EQ(out.back(), '\n');
}
