// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/joke/joke_layout.hpp"

#include "bagwiz/core/tui/width.hpp"
#include "bagwiz/core/tui/word_wrap.hpp"

#include <algorithm>
#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

namespace bagwiz::core::joke
{

namespace
{

// Blank columns between the face and the joke text.
constexpr std::string_view kGap = "   ";

// Bounds on the joke text column. The lower bound keeps text readable even
// on a terminal narrower than the face (the line then unavoidably exceeds
// the terminal, but the text stays a sensible shape). The upper bound keeps
// a comfortable measure on very wide terminals instead of one long line.
constexpr int kMinTextColumns = 16;
constexpr int kMaxTextColumns = 56;

// One spare column so a full-width line never lands exactly on the terminal
// edge, which some terminals treat as an overflow and hard-wrap.
constexpr int kRightMargin = 1;

// Block-character "Wojak" meme. Supplied verbatim, with the left-edge
// horizontal-line padding removed (the leading box-drawing dashes are
// replaced by plain spaces so the figure has no line down its left side).
constexpr std::string_view kFaceArt =
  R"FACE(       ▄▀▀▀▀▀▀▀▀▀▀▄▄
    ▄▀▀░░░░░░░░░░░░░▀▄
  ▄▀░░░░░░░░░░░░░░░░░░▀▄
  █░░░░░░░░░░░░░░░░░░░░░▀▄
 ▐▌░░░░░░░░▄▄▄▄▄▄▄░░░░░░░▐▌
 █░░░░░░░░░░░▄▄▄▄░░▀▀▀▀▀░░█
▐▌░░░░░░░▀▀▀▀░░░░░▀▀▀▀▀░░░▐▌
█░░░░░░░░░▄▄▀▀▀▀▀░░░░▀▀▀▀▄░█
█░░░░░░░░░░░░░░░░▀░░░▐░░░░░▐▌
▐▌░░░░░░░░░▐▀▀██▄░░░░░░▄▄▄░▐▌
 █░░░░░░░░░░░▀▀▀░░░░░░▀▀██░░█
 ▐▌░░░░▄░░░░░░░░░░░░░▌░░░░░░█
  ▐▌░░▐░░░░░░░░░░░░░░▀▄░░░░░█
   █░░░▌░░░░░░░░▐▀░░░░▄▀░░░▐▌
   ▐▌░░▀▄░░░░░░░░▀░▀░▀▀░░░▄▀
   ▐▌░░▐▀▄░░░░░░░░░░░░░░░░█
   ▐▌░░░▌░▀▄░░░░▀▀▀▀▀▀░░░█
   █░░░▀░░░░▀▄░░░░░░░░░░▄▀
  ▐▌░░░░░░░░░░▀▄░░░░░░▄▀
 ▄▀░░░▄▀░░░░░░░░▀▀▀▀█▀
▀░░░▄▀░░░░░░░░░░▀░░░▀▀▀▀▄▄▄▄▄)FACE";

std::vector<std::string> split_lines(std::string_view text)
{
  std::vector<std::string> lines;
  std::string current;
  for (const char c : text) {
    if (c == '\n') {
      lines.push_back(current);
      current.clear();
    } else {
      current.push_back(c);
    }
  }
  lines.push_back(current);
  return lines;
}

}  // namespace

// `text` is a std::string_view: cheap to copy, so by value is the
// guideline-preferred form (F.16).
std::string render_joke(std::string_view text, int terminal_columns)
{
  const std::vector<std::string> face = split_lines(kFaceArt);

  int face_width = 0;
  for (const auto & line : face) {
    face_width = std::max(face_width, core::tui::display_width(line));
  }

  // Wrap the joke within its own column only: the column starts after the
  // face plus the gap, so subtracting both (and a spare margin) from the
  // terminal width gives the room the text has before the terminal would
  // hard-wrap it back under the face.
  const int gap_width = core::tui::display_width(kGap);
  const int available = terminal_columns - face_width - gap_width - kRightMargin;
  const int text_width = std::clamp(available, kMinTextColumns, kMaxTextColumns);
  const std::vector<std::string> joke = core::tui::word_wrap(text, text_width);

  // Vertically center the joke lines against the face block so the text sits
  // beside the middle of the face rather than its top.
  const std::size_t start_row = face.size() > joke.size() ? (face.size() - joke.size()) / 2 : 0;

  const auto append_text_column = [&](std::string & out, std::size_t joke_index) {
    out.append(kGap).append(joke[joke_index]);
  };

  std::string out;
  for (std::size_t row = 0; row < face.size(); ++row) {
    const std::string & face_line = face[row];
    const bool has_text = row >= start_row && (row - start_row) < joke.size();
    if (!has_text) {
      // No text on this row: emit the face line with no trailing padding.
      out.append(face_line).append("\n");
      continue;
    }
    const int pad = std::max(0, face_width - core::tui::display_width(face_line));
    out.append(face_line).append(static_cast<std::size_t>(pad), ' ');
    append_text_column(out, row - start_row);
    out.append("\n");
  }

  // Any joke lines beyond the face height (only for unusually long jokes):
  // align them under the text column.
  for (std::size_t i = face.size() - start_row; i < joke.size(); ++i) {
    out.append(static_cast<std::size_t>(face_width), ' ');
    append_text_column(out, i);
    out.append("\n");
  }
  return out;
}

}  // namespace bagwiz::core::joke
