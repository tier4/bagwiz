// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/core/tui/word_wrap.hpp"

#include "bagwiz/core/tui/width.hpp"

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

namespace bagwiz::core::tui
{

namespace
{

bool is_space(char c) noexcept
{
  return c == ' ' || c == '\t';
}

// Greedily pack the words of one '\n'-free paragraph onto lines no wider than
// max_cols and append them to `out`. Whitespace runs between words collapse to
// a single space and a separating space costs one column. A word wider than
// max_cols still lands on its own line (it overflows rather than being split).
// A paragraph with no words appends a single empty line so blank separators
// survive.
void wrap_paragraph(std::string_view paragraph, int max_cols, std::vector<std::string> & out)
{
  std::string current;
  int current_w = 0;
  bool has_word = false;

  std::size_t i = 0;
  while (i < paragraph.size()) {
    while (i < paragraph.size() && is_space(paragraph[i])) {
      ++i;  // skip the whitespace run between words
    }
    if (i >= paragraph.size()) {
      break;
    }
    const std::size_t word_start = i;
    while (i < paragraph.size() && !is_space(paragraph[i])) {
      ++i;
    }
    const std::string_view word = paragraph.substr(word_start, i - word_start);
    const int word_w = display_width(word);

    if (!has_word) {
      current.assign(word);
      current_w = word_w;
      has_word = true;
    } else if (current_w + 1 + word_w <= max_cols) {
      current.push_back(' ');
      current.append(word);
      current_w += 1 + word_w;
    } else {
      out.push_back(current);
      current.assign(word);
      current_w = word_w;
    }
  }

  out.push_back(current);  // the last line, or "" when the paragraph had no words
}

}  // namespace

std::vector<std::string> word_wrap(std::string_view text, int max_cols)
{
  if (max_cols <= 0) {
    return {std::string(text)};
  }

  std::vector<std::string> out;
  std::size_t start = 0;
  while (true) {
    const std::size_t nl = text.find('\n', start);
    const std::size_t len = (nl == std::string_view::npos) ? std::string_view::npos : nl - start;
    wrap_paragraph(text.substr(start, len), max_cols, out);
    if (nl == std::string_view::npos) {
      break;
    }
    start = nl + 1;
  }
  return out;
}

}  // namespace bagwiz::core::tui
