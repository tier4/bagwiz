// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "bagwiz/commands/help_formatter.hpp"

#include "CLI/CLI.hpp"
#include "bagwiz/core/tui/layout.hpp"
#include "bagwiz/core/tui/width.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace
{

using bagwiz::commands::HelpFormatter;
using bagwiz::core::tui::display_width;

// The description column starts at the formatter's column width (CLI11's
// default name column). The tests fix this so the available width is known
// independently of the real terminal size.
constexpr std::size_t kColumnWidth = 30;

std::vector<std::string> split_lines(const std::string & text)
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

TEST(HelpFormatter, WrapsLongDescriptionToTheDescriptionColumn)
{
  const int cols = bagwiz::core::tui::query_terminal_size().cols;
  if (cols < 60) {
    GTEST_SKIP() << "terminal too narrow for a deterministic wrap assertion";
  }

  HelpFormatter fmt;
  // Pin the available description width to a known value regardless of the
  // live terminal width: max_cols = cols - column_width = 30.
  fmt.column_width(static_cast<std::size_t>(cols) - kColumnWidth);

  CLI::App app;
  std::string sink;
  CLI::Option * opt = app.add_option(
    "--example", sink, "alpha bravo charlie delta echo foxtrot golf hotel india juliet kilo lima");

  const std::string wrapped = fmt.make_option_desc(opt);
  const std::vector<std::string> lines = split_lines(wrapped);

  ASSERT_GT(lines.size(), 1U) << "long description should wrap onto multiple lines";
  for (const auto & line : lines) {
    EXPECT_LE(display_width(line), static_cast<int>(kColumnWidth))
      << "line exceeds width: " << line;
  }
}

TEST(HelpFormatter, WrapsSubcommandDescriptionToTheDescriptionColumn)
{
  const int cols = bagwiz::core::tui::query_terminal_size().cols;
  if (cols < 60) {
    GTEST_SKIP() << "terminal too narrow for a deterministic wrap assertion";
  }

  HelpFormatter fmt;
  // Pin the description column so the available width is a known 30 columns
  // regardless of the live terminal width.
  const std::size_t name_col = static_cast<std::size_t>(cols) - kColumnWidth;
  fmt.column_width(name_col);

  CLI::App app;
  CLI::App * sub = app.add_subcommand(
    "geo", "alpha bravo charlie delta echo foxtrot golf hotel india juliet kilo lima");

  const std::string rendered = fmt.make_subcommand(sub);

  // The wrapped description's continuation lines are indented to the
  // description column (the formatter's column width).
  const std::string hanging_indent = "\n" + std::string(name_col, ' ');
  EXPECT_NE(rendered.find(hanging_indent), std::string::npos)
    << "subcommand description should wrap with a hanging indent: " << rendered;
}

TEST(HelpFormatter, ShortDescriptionIsUnchanged)
{
  HelpFormatter fmt;
  CLI::App app;
  std::string sink;
  CLI::Option * opt = app.add_option("--flagopt", sink, "Short and sweet.");

  // A short description never wraps, on any terminal width.
  EXPECT_EQ(fmt.make_option_desc(opt), "Short and sweet.");
}

TEST(HelpFormatter, TrailingNewlineDoesNotAddBlankLine)
{
  HelpFormatter fmt;
  CLI::App app;
  std::string sink;
  CLI::Option * opt = app.add_option("--x", sink, "Has a trailing newline.\n");

  const std::string result = fmt.make_option_desc(opt);
  EXPECT_EQ(result, "Has a trailing newline.");
  EXPECT_TRUE(result.empty() || result.back() != '\n');
}

TEST(HelpFormatter, EmptyDescriptionStaysEmpty)
{
  HelpFormatter fmt;
  CLI::App app;
  std::string sink;
  CLI::Option * opt = app.add_option("--undocumented", sink);

  EXPECT_EQ(fmt.make_option_desc(opt), "");
}

TEST(HelpFormatter, HelpOutputIndentsContinuationLines)
{
  const int cols = bagwiz::core::tui::query_terminal_size().cols;
  if (cols < 60) {
    GTEST_SKIP() << "terminal too narrow for a deterministic wrap assertion";
  }

  CLI::App app{"A tool"};
  app.formatter(std::make_shared<HelpFormatter>());  // default column width (30)

  // A description comfortably longer than max_cols (cols - 30) so it wraps.
  std::string desc;
  while (static_cast<int>(desc.size()) < cols * 2) {
    desc += "lorem ipsum dolor ";
  }
  std::string sink;
  app.add_option("--verbose", sink, desc);

  const std::string help = app.help();

  // CLI11's format_help() re-indents every line after an embedded '\n' to the
  // description column, so a wrapped continuation appears as newline + spaces.
  const std::string hanging_indent = "\n" + std::string(kColumnWidth, ' ');
  EXPECT_NE(help.find(hanging_indent), std::string::npos)
    << "expected wrapped continuation lines indented to the description column";
}

}  // namespace
