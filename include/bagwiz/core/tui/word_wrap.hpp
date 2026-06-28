// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__TUI__WORD_WRAP_HPP_
#define BAGWIZ__CORE__TUI__WORD_WRAP_HPP_

#include <string>
#include <string_view>
#include <vector>

namespace bagwiz::core::tui
{

// Greedily word-wrap prose `text` into lines whose `display_width` is
// <= max_cols, breaking only at whitespace so a word is never split.
//
// Unlike wrap_to_width (which breaks on codepoint boundaries to preserve a
// leading indent for YAML), this reflows running text the way a help renderer
// or a centered caption wants it:
//   * Breaks only at ASCII whitespace (spaces/tabs); words stay intact.
//   * Inter-word whitespace is collapsed to a single space; leading and
//     trailing whitespace on each paragraph is dropped.
//   * Explicit '\n' in `text` is a hard break: each paragraph wraps
//     independently and blank paragraphs survive as empty lines.
//   * A single word wider than max_cols overflows onto its own line rather
//     than being chopped (URLs and long paths stay intact).
//   * Width is measured with display_width, so CJK/wide glyphs and CSI
//     escapes are accounted for correctly.
//
// Lines carry no indentation; callers add any hanging indent themselves.
// Always returns at least one line (an empty input yields {""}).
//
// `max_cols <= 0` disables wrapping and returns {string(text)} unchanged.
std::vector<std::string> word_wrap(std::string_view text, int max_cols);

}  // namespace bagwiz::core::tui

#endif  // BAGWIZ__CORE__TUI__WORD_WRAP_HPP_
