// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__CORE__JOKE__JOKE_LAYOUT_HPP_
#define BAGWIZ__CORE__JOKE__JOKE_LAYOUT_HPP_

#include <string>
#include <string_view>

namespace bagwiz::core::joke
{

// Lay out the Wojak "saying" `text`: the block-character face on the
// left, with the joke placed to its right and vertically centered against
// the face. The joke wraps within its own column only — the wrap width is
// derived from `terminal_columns` (face width + gap subtracted) so that no
// rendered line exceeds the terminal and the text never spills under the
// face. Pass the live terminal width (e.g. tui::query_terminal_size().cols).
// Returns a multi-line string terminated by a newline; never throws.
[[nodiscard]] std::string render_joke(std::string_view text, int terminal_columns);

}  // namespace bagwiz::core::joke

#endif  // BAGWIZ__CORE__JOKE__JOKE_LAYOUT_HPP_
