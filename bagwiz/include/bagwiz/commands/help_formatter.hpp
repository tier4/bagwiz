// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__HELP_FORMATTER_HPP_
#define BAGWIZ__COMMANDS__HELP_FORMATTER_HPP_

#include "CLI/CLI.hpp"

#include <string>

namespace bagwiz::commands
{

// CLI11 formatter that word-wraps option, positional, and subcommand
// descriptions to the current terminal width.
//
// CLI11's default leaves a long description on a single line and lets the
// terminal soft-wrap it back to column 0, breaking the alignment with the
// description column. These overrides re-wrap the description at word
// boundaries to (terminal width - column width) and emit embedded newlines;
// CLI11's format_help() then re-indents every continuation line under the
// description column, giving a readable hanging indent.
//
// Install once on the top-level App before adding subcommands; CLI11 hands the
// formatter down to every subcommand at construction, so nested commands
// inherit it automatically.
class HelpFormatter : public CLI::Formatter
{
public:
  std::string make_option_desc(const CLI::Option * opt) const override;
  std::string make_subcommand(const CLI::App * sub) const override;

private:
  // Word-wrap `description` to the current terminal width, returning it with
  // embedded newlines so format_help() re-indents continuation lines under the
  // description column. Returns the text unchanged on very narrow terminals.
  std::string wrap_description(const std::string & description) const;
};

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__HELP_FORMATTER_HPP_
