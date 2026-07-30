// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/commands/completion.hpp"
#include "bagwiz/commands/help_formatter.hpp"
#include "bagwiz/commands/parse_error.hpp"
#include "bagwiz/core/base/logging.hpp"

#include <exception>
#include <memory>
#include <string>
#include <vector>

namespace
{
constexpr const char * kVersion = "0.1.0";
constexpr const char * kMainLogger = "bagwiz.main";

// CLI11's default failure message prints only the long form of a required
// option ("--input is required"). Rewrite it so options with a short form
// are reported as "-i/--input is required", then append the standard help
// hint. This mirrors CLI11's FailureMessage::simple but applies
// rewrite_parse_error to the diagnostic.
std::string failure_message(const CLI::App * app, const CLI::Error & e)
{
  std::string header = bagwiz::commands::rewrite_parse_error(*app, e) + "\n";

  std::vector<std::string> names;
  if (app->get_help_ptr() != nullptr) {
    names.push_back(app->get_help_ptr()->get_name());
  }
  if (app->get_help_all_ptr() != nullptr) {
    names.push_back(app->get_help_all_ptr()->get_name());
  }

  if (!names.empty()) {
    header += "Run with " + CLI::detail::join(names, " or ") + " for more information.\n";
  }

  return header;
}
}  // namespace

int main(int argc, char ** argv) noexcept
{
  try {
    try {
      bagwiz::core::init_logging();
      if (bagwiz::commands::is_completion_request(argc, argv)) {
        return bagwiz::commands::run_completion_request(argc, argv);
      }
    } catch (const std::exception & e) {
      BAGWIZ_LOG_FATAL(kMainLogger, "Unhandled exception during startup: %s", e.what());
      return 1;
    }

    CLI::App app{
      "bagwiz - Fast CLI for analyzing, processing, and extracting data from ROS 2 rosbags "
      "offline, without a ROS graph"};
    app.set_version_flag("--version", kVersion);
    app.require_subcommand(1);
    // Word-wrap option and subcommand descriptions to the terminal width so
    // continuation lines align under the description column. Set before adding
    // subcommands: CLI11 hands the formatter down to each subcommand at
    // construction.
    app.formatter(std::make_shared<bagwiz::commands::HelpFormatter>());
    // Rewrite parse errors so required options report both short and long
    // forms (e.g. "-i/--input is required"). Subcommands inherit this callback
    // from the top-level App.
    app.failure_message(failure_message);

    const auto & registry = bagwiz::commands::Registry::instance();
    // Selected is set by the top-level subcommand callback; nested subcommands
    // store their own state on the Command instance. run() fires once after
    // parsing completes so parent and child callbacks can both observe args
    // before the command executes.
    bagwiz::commands::Command * selected = nullptr;
    for (const auto & cmd : registry.all()) {
      auto * sub = app.add_subcommand(std::string(cmd->name()), std::string(cmd->description()));
      // An empty group name keeps the subcommand fully functional while
      // omitting it from the top-level --help listing (used for hidden
      // easter-egg commands).
      if (cmd->hidden()) {
        sub->group("");
      }
      cmd->configure(*sub);
      sub->callback([&selected, raw = cmd.get()]() { selected = raw; });
    }

    try {
      CLI11_PARSE(app, argc, argv);
    } catch (const std::exception & e) {
      BAGWIZ_LOG_FATAL(kMainLogger, "Unhandled exception during argument parsing: %s", e.what());
      return 1;
    }

    if (!selected) {
      // CLI11 handled --help/--version or required_subcommand already printed
      // an error; nothing further to do.
      return 0;
    }

    try {
      return selected->run();
    } catch (const std::exception & e) {
      BAGWIZ_LOG_FATAL(kMainLogger, "Command failed: %s", e.what());
      return 1;
    }
  } catch (...) {
    // Last-resort barrier so exceptions never escape main() (E.6 / E.12).
    // The inner try/catch blocks above provide stage-specific diagnostics;
    // this only fires for non-std::exception throws (rare) or for failures
    // in code outside the inner blocks (CLI::App construction, registry
    // iteration). Logging may itself throw here, so this catch must not.
    return 1;
  }
}
