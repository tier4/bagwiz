// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__TUI__WIDTH_HPP_
#define BAGWIZ__CORE__TUI__WIDTH_HPP_

#include <string>
#include <string_view>
#include <vector>

namespace bagwiz::core::tui
{

// Display width of `s` measured in terminal columns.
//
// Rules:
//   * ASCII printable (0x20..0x7E): width 1 per byte.
//   * ASCII control bytes (0x00..0x1F, 0x7F) other than ESC: width 0;
//     callers should strip newlines/tabs before calling.
//   * CSI sequences starting with ESC '[' and ending at a byte in the
//     range 0x40..0x7E (e.g. SGR `\x1B[31m`, cursor `\x1B[H`): width 0.
//   * UTF-8 codepoints in the simplified CJK-wide ranges below: width 2.
//     - U+1100..U+115F   Hangul Jamo
//     - U+2E80..U+303E   CJK Radicals / Kangxi
//     - U+3041..U+33FF   Hiragana / Katakana / CJK Symbols
//     - U+3400..U+4DBF   CJK Unified Ideographs Extension A
//     - U+4E00..U+9FFF   CJK Unified Ideographs
//     - U+A000..U+A4CF   Yi Syllables
//     - U+AC00..U+D7A3   Hangul Syllables
//     - U+F900..U+FAFF   CJK Compatibility Ideographs
//     - U+FE30..U+FE4F   CJK Compatibility Forms
//     - U+FF00..U+FF60   Fullwidth Forms (excluding halfwidth Katakana)
//     - U+FFE0..U+FFE6   Fullwidth signs
//   * Any other UTF-8 codepoint: width 1.
//   * An incomplete trailing UTF-8 byte sequence is silently ignored.
int display_width(std::string_view s) noexcept;

// Return a prefix of `s` whose `display_width` is <= max_cols. The cut
// never falls in the middle of a UTF-8 codepoint or a CSI escape:
//   * If a wide codepoint would push the width past max_cols, it is not
//     included in the result.
//   * A CSI sequence is treated atomically: either fully included or
//     fully excluded. An incomplete trailing CSI sequence is dropped.
//   * `max_cols <= 0` returns "".
std::string truncate_to_width(std::string_view s, int max_cols);

// Wrap `s` into one or more lines whose `display_width` is <= max_cols.
// Continuation lines (every line after the first) are prefixed with the
// leading ASCII whitespace (spaces/tabs) of `s` so wrapped YAML preserves
// its visual nesting. CSI sequences are treated atomically and attach to
// the segment currently accumulating; UTF-8 codepoints never split.
//
// Special cases:
//   * `max_cols <= 0` returns {string(s)} unchanged (no wrap).
//   * An empty input returns {""} (one empty line) so blank YAML
//     separators survive a round-trip.
//   * If the leading indent is >= `max_cols` (so a continuation line
//     would have no room for content), the indent is dropped on
//     continuation lines.
std::vector<std::string> wrap_to_width(std::string_view s, int max_cols);

}  // namespace bagwiz::core::tui

#endif  // BAGWIZ__CORE__TUI__WIDTH_HPP_
