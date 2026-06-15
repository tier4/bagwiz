// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/commands/topic_omit.hpp"
#include "bagwiz/core/logging.hpp"

#include <string_view>

namespace bagwiz::commands
{

namespace
{
constexpr const char * kLogger = "bagwiz.cmd.topic";
}  // namespace

// `bagwiz topic` is a command group for topic-level bag surgery. Ships `omit`
// (drop selected topics); the group leaves room for further topic operations.
class TopicCommand : public Command
{
public:
  [[nodiscard]] std::string_view name() const override { return "topic"; }
  [[nodiscard]] std::string_view description() const override
  {
    return "Topic-level bag operations";
  }

  void configure(CLI::App & app) override
  {
    app.require_subcommand(1);
    configure_omit(app);
  }

  int run() override
  {
    switch (selected_) {
      case Subcommand::kOmit:
        return run_topic_omit(omit_args_);
      case Subcommand::kNone:
        BAGWIZ_LOG_ERROR(kLogger, "no subcommand selected");
        return 1;
    }
    return 1;
  }

private:
  enum class Subcommand { kNone, kOmit };
  Subcommand selected_ = Subcommand::kNone;

  TopicOmitArgs omit_args_;

  void configure_omit(CLI::App & app)
  {
    auto * sub = app.add_subcommand(
      "omit",
      "Remove topics from a rosbag, copying every other topic verbatim. Each "
      "<topic> is a literal name or a '*' glob (e.g. /sensing/*); '*' matches any "
      "run of characters, including '/'.");
    sub->add_option("input", omit_args_.input_path, "Input ROS 2 rosbag (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    sub
      ->add_option(
        "topics", omit_args_.topics,
        "Topic selector(s) to remove. A literal topic name or a '*' glob. Repeat for several.")
      ->required();
    sub->add_option(
      "-o,--output", omit_args_.output_path,
      "Write the result to this new bag instead of rewriting <input> in place.");
    sub->add_flag(
      "--overwrite", omit_args_.overwrite,
      "Replace an existing -o/--output path. Without it, an existing output path stops the run.");
    sub->footer(
      "Removed topics disappear entirely from the output — both their messages and their\n"
      "topic declarations. Messages are copied verbatim; no deserialization is performed.\n"
      "A selector that matches no topic stops the run before anything is written.\n"
      "Without -o, <input> is rewritten in place via an atomic tmp-swap that preserves its\n"
      "storage format and layout (input is both source and destination); with -o, <input>\n"
      "is left untouched.");
    sub->callback([this]() { selected_ = Subcommand::kOmit; });
  }
};

BAGWIZ_REGISTER_COMMAND(TopicCommand)

}  // namespace bagwiz::commands
