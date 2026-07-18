// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/help_formatter.hpp"

#include "bagwiz/core/tui/layout.hpp"
#include "bagwiz/core/tui/word_wrap.hpp"

#include <cstddef>
#include <sstream>
#include <string>
#include <vector>

namespace bagwiz::commands
{

namespace
{
// Below this many columns for the description, a hanging indent leaves so
// little room that the wrap looks worse than the terminal's own soft-wrap;
// leave the text unwrapped on very narrow terminals.
constexpr int kMinDescWidth = 20;
}  // namespace

std::string HelpFormatter::wrap_description(const std::string & description) const
{
  std::string text = description;
  // Strip trailing newlines so a "desc\n" description does not wrap into a
  // spurious blank continuation line once format_help() re-indents.
  while (!text.empty() && text.back() == '\n') {
    text.pop_back();
  }
  if (text.empty()) {
    return text;
  }

  const int columns = bagwiz::core::tui::query_terminal_size().cols;
  const int max_cols = columns - static_cast<int>(get_column_width());
  if (max_cols < kMinDescWidth) {
    return text;  // narrow terminal: don't fight the soft-wrap
  }

  // format_help() re-indents every line after an embedded '\n' to the
  // description column, so joining the wrapped lines with '\n' yields the
  // hanging indent.
  const std::vector<std::string> lines = bagwiz::core::tui::word_wrap(text, max_cols);
  std::string out;
  for (std::size_t i = 0; i < lines.size(); ++i) {
    if (i != 0) {
      out.push_back('\n');
    }
    out.append(lines[i]);
  }
  return out;
}

std::string HelpFormatter::make_option_desc(const CLI::Option * opt) const
{
  return wrap_description(opt->get_description());
}

std::string HelpFormatter::make_subcommand(const CLI::App * sub) const
{
  // Mirror CLI11's default make_subcommand (name [+ REQUIRED] in the left
  // column) but word-wrap the description so its continuation lines align
  // under the description column instead of soft-wrapping to column 0.
  std::string name = sub->get_display_name(true);
  if (sub->get_required()) {
    name += " " + get_label("REQUIRED");
  }
  std::ostringstream out;
  CLI::detail::format_help(out, name, wrap_description(sub->get_description()), get_column_width());
  return out.str();
}

}  // namespace bagwiz::commands
