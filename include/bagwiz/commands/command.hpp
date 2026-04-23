// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__COMMAND_HPP_
#define BAGWIZ__COMMANDS__COMMAND_HPP_

#include <memory>
#include <string_view>
#include <vector>

namespace CLI
{
class App;
}  // namespace CLI

namespace bagwiz::commands
{

// Base class for CLI subcommands. Each command is a plugin: it declares its
// arguments via configure() at startup, and run() is invoked after parsing.
// Concrete implementations register themselves with BAGWIZ_REGISTER_COMMAND.
class Command
{
public:
  virtual ~Command() = default;

  // Subcommand name as typed on the CLI, e.g. "topic", "comp".
  virtual std::string_view name() const = 0;

  // One-line description shown in --help.
  virtual std::string_view description() const = 0;

  // Declare arguments/options/flags on `app`. Called once at startup.
  virtual void configure(CLI::App & app) = 0;

  // Execute. Return value becomes the process exit code.
  virtual int run() = 0;
};

// Process-wide registry of command plugins. Commands insert themselves at
// static-init time via BAGWIZ_REGISTER_COMMAND; main() walks the list and
// wires each into the top-level CLI::App.
class Registry
{
public:
  static Registry & instance();

  void add(std::unique_ptr<Command> cmd);
  const std::vector<std::unique_ptr<Command>> & all() const { return commands_; }

private:
  Registry() = default;
  std::vector<std::unique_ptr<Command>> commands_;
};

// Register a Command subclass. Place this macro at namespace scope in the
// command's .cpp file. The command .cpp must be part of the executable
// target (not a library) so the registrar is not stripped by the linker.
#define BAGWIZ_REGISTER_COMMAND(CommandType)                                            \
  namespace                                                                             \
  {                                                                                     \
  struct CommandType##_Registrar                                                        \
  {                                                                                     \
    CommandType##_Registrar()                                                           \
    {                                                                                   \
      ::bagwiz::commands::Registry::instance().add(std::make_unique<CommandType>());    \
    }                                                                                   \
  };                                                                                    \
  [[maybe_unused]] static const CommandType##_Registrar kBagcliRegistrar_##CommandType; \
  }

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__COMMAND_HPP_
