// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/core/broken_bag.hpp"
#include "bagwiz/core/logging.hpp"

#include <fmt/core.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace bagwiz::commands
{

namespace
{

constexpr const char * kLogger = "bagwiz.cmd.check";

// Trim ASCII whitespace from both ends and lowercase the result.
std::string normalize_answer(std::string s)
{
  const auto not_space = [](unsigned char c) { return std::isspace(c) == 0; };
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
  s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
  for (auto & c : s) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return s;
}

// Ask the user once whether to delete the broken bags. The prompt goes to
// stderr so stdout stays a clean, one-path-per-line list. When stdin is not a
// TTY (piped / redirected), returns false without prompting so a pipeline
// never deletes anything without an explicit --rm.
bool confirm_delete(std::size_t count)
{
  if (::isatty(STDIN_FILENO) == 0) {
    return false;
  }
  fmt::print(stderr, "Delete {} broken rosbag(s)? [y/N] ", count);
  std::fflush(stderr);

  std::string line;
  if (!std::getline(std::cin, line)) {
    return false;  // EOF / read error: treat as "no".
  }
  const std::string answer = normalize_answer(std::move(line));
  return answer == "y" || answer == "yes";
}

}  // namespace

// `bagwiz check` is a command group for rosbag integrity checks.
//
// Subcommands
// -----------
//   broken  Scan <input> (a single bag or a directory walked recursively) for
//           rosbags whose storage container is corrupt / unreadable, list
//           them, and optionally delete them (interactively, or with --rm).
class CheckCommand : public Command
{
public:
  [[nodiscard]] std::string_view name() const override { return "check"; }
  [[nodiscard]] std::string_view description() const override { return "Rosbag integrity checks"; }

  void configure(CLI::App & app) override
  {
    app.require_subcommand(1);
    configure_broken(app);
  }

  int run() override
  {
    switch (selected_) {
      case Subcommand::kBroken:
        return run_broken();
      case Subcommand::kNone:
        BAGWIZ_LOG_ERROR(kLogger, "no subcommand selected");
        return 1;
    }
    return 1;
  }

private:
  enum class Subcommand { kNone, kBroken };
  Subcommand selected_ = Subcommand::kNone;

  struct BrokenArgs
  {
    std::filesystem::path input_path;
    bool remove = false;
    bool deep = false;
  } broken_args_;

  void configure_broken(CLI::App & app)
  {
    auto * sub = app.add_subcommand(
      "broken",
      "Find rosbags whose storage container is corrupt / unreadable and, with --rm or after "
      "confirmation, delete them. A mismatch between metadata.yaml and the actual records is not "
      "treated as broken.");
    sub
      ->add_option(
        "input", broken_args_.input_path,
        "Bag path or directory to scan. A directory is walked recursively and the check is applied "
        "to every rosbag found within it.")
      ->required()
      ->check(CLI::ExistingPath);
    sub->add_flag(
      "--rm", broken_args_.remove,
      "Delete every broken bag without prompting. Without this flag the broken bags are listed and "
      "you are asked once before anything is deleted.");
    sub->add_flag(
      "--deep", broken_args_.deep,
      "Thorough mode: stream every message to end-of-file (without decoding) to catch payload "
      "corruption a structural check cannot see. Reads the whole bag, so it is much slower.");
    sub->callback([this]() { selected_ = Subcommand::kBroken; });
  }

  int run_broken()
  {
    const auto & args = broken_args_;

    const auto units = core::discover_bags(args.input_path);
    if (units.empty()) {
      BAGWIZ_LOG_ERROR(kLogger, "No rosbag found at %s", args.input_path.c_str());
      return 1;
    }

    BAGWIZ_LOG_INFO(
      kLogger, "Scanning %zu rosbag(s) for corruption%s...", units.size(),
      args.deep ? " (deep)" : "");

    std::vector<core::BagUnit> broken;
    for (const auto & unit : units) {
      if (const auto reason = core::diagnose_bag(unit.path, args.deep)) {
        // The reason is a diagnostic -> stderr; the path is the command's
        // data output and is written to stdout below.
        BAGWIZ_LOG_WARN(kLogger, "broken: %s (%s)", unit.path.c_str(), reason->c_str());
        broken.push_back(unit);
      }
    }

    if (broken.empty()) {
      BAGWIZ_LOG_INFO(kLogger, "No broken rosbags found (%zu scanned).", units.size());
      return 0;
    }

    // The broken-bag list is the command's data output: one path per line on
    // stdout so it pipes cleanly into xargs, rm, and similar tools.
    for (const auto & unit : broken) {
      fmt::print(stdout, "{}\n", unit.path.string());
    }
    if (std::fflush(stdout) != 0) {
      BAGWIZ_LOG_ERROR(kLogger, "Failed to write the broken-bag list to stdout");
      return 1;
    }

    if (!args.remove && !confirm_delete(broken.size())) {
      BAGWIZ_LOG_INFO(
        kLogger, "Left %zu broken rosbag(s) in place. Re-run with --rm to delete.", broken.size());
      return 0;
    }

    std::size_t deleted = 0;
    std::size_t failed = 0;
    for (const auto & unit : broken) {
      if (const auto ec = core::delete_bag(unit)) {
        BAGWIZ_LOG_ERROR(
          kLogger, "Failed to delete %s: %s", unit.path.c_str(), ec.message().c_str());
        ++failed;
      } else {
        BAGWIZ_LOG_INFO(kLogger, "deleted %s", unit.path.c_str());
        ++deleted;
      }
    }

    if (failed > 0) {
      BAGWIZ_LOG_ERROR(
        kLogger, "Deleted %zu of %zu broken rosbag(s); %zu failed.", deleted, broken.size(),
        failed);
      return 1;
    }
    BAGWIZ_LOG_INFO(kLogger, "Deleted %zu broken rosbag(s).", deleted);
    return 0;
  }
};

BAGWIZ_REGISTER_COMMAND(CheckCommand)

}  // namespace bagwiz::commands
