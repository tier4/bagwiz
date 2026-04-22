// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagcli/commands/command.hpp"
#include "bagcli/core/logging.hpp"

#include <exception>
#include <string>

namespace
{
constexpr const char * kVersion = "0.1.0";
constexpr const char * kMainLogger = "bagcli.main";
}  // namespace

int main(int argc, char ** argv)
{
  bagcli::core::init_logging();

  CLI::App app{"bagcli - Fast CLI for analyzing and processing ROS 2 rosbags"};
  app.set_version_flag("--version", kVersion);
  app.require_subcommand(1);

  auto & registry = bagcli::commands::Registry::instance();
  // Selected is set by the top-level subcommand callback; nested subcommands
  // store their own state on the Command instance. run() fires once after
  // parsing completes so parent and child callbacks can both observe args
  // before the command executes.
  bagcli::commands::Command * selected = nullptr;
  for (const auto & cmd : registry.all()) {
    auto * sub = app.add_subcommand(std::string(cmd->name()), std::string(cmd->description()));
    cmd->configure(*sub);
    sub->callback([&selected, raw = cmd.get()]() { selected = raw; });
  }

  try {
    CLI11_PARSE(app, argc, argv);
  } catch (const std::exception & e) {
    BAGCLI_LOG_FATAL(kMainLogger, "Unhandled exception during argument parsing: %s", e.what());
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
    BAGCLI_LOG_FATAL(kMainLogger, "Command failed: %s", e.what());
    return 1;
  }
}
