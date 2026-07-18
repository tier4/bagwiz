// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/core/base/logging.hpp"
#include "bagwiz/core/joke/joke_layout.hpp"
#include "bagwiz/core/joke/jokes.hpp"
#include "bagwiz/core/tui/layout.hpp"

#include <exception>
#include <iostream>
#include <string_view>
#include <vector>

namespace bagwiz::commands
{

namespace
{
constexpr const char * kLogger = "bagwiz.cmd.joke";
}  // namespace

// `bagwiz joke` is an undocumented easter egg: it prints a random ROS joke
// (embedded into the binary at build time from src/core/joke/jokes.json),
// shown beside a Wojak. It is hidden from --help and from shell
// completion (see hidden()); only someone who already knows the command
// exists can run it.
class JokeCommand : public Command
{
public:
  [[nodiscard]] std::string_view name() const override { return "joke"; }
  [[nodiscard]] std::string_view description() const override { return "Tell a ROS joke"; }
  [[nodiscard]] bool hidden() const override { return true; }

  void configure(CLI::App & app) override
  {
    // No arguments: `bagwiz joke` takes nothing.
    (void)app;
  }

  int run() override
  {
    try {
      const auto jokes = core::joke::load_jokes();
      const auto terminal = core::tui::query_terminal_size();
      std::cout << core::joke::render_joke(core::joke::random_joke(jokes), terminal.cols);
      return 0;
    } catch (const std::exception & e) {
      BAGWIZ_LOG_ERROR(kLogger, "Could not tell a joke: %s", e.what());
      return 1;
    }
  }
};

BAGWIZ_REGISTER_COMMAND(JokeCommand)

}  // namespace bagwiz::commands
