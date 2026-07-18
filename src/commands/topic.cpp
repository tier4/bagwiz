// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/commands/topic_drop.hpp"
#include "bagwiz/commands/topic_keep.hpp"
#include "bagwiz/commands/topic_rename.hpp"
#include "bagwiz/core/logging.hpp"

#include <string_view>

namespace bagwiz::commands
{

namespace
{
constexpr const char * kLogger = "bagwiz.cmd.topic";
}  // namespace

// `bagwiz topic` is a command group for topic-level bag surgery. Ships `drop`
// (drop selected topics), `keep` (its inverse — keep only selected topics), and
// `rename` (rename one topic); the group leaves room for further topic
// operations.
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
    configure_drop(app);
    configure_keep(app);
    configure_rename(app);
  }

  int run() override
  {
    switch (selected_) {
      case Subcommand::kDrop:
        return run_topic_drop(drop_args_);
      case Subcommand::kKeep:
        return run_topic_keep(keep_args_);
      case Subcommand::kRename:
        return run_topic_rename(rename_args_);
      case Subcommand::kNone:
        BAGWIZ_LOG_ERROR(kLogger, "no subcommand selected");
        return 1;
    }
    return 1;
  }

private:
  enum class Subcommand { kNone, kDrop, kKeep, kRename };
  Subcommand selected_ = Subcommand::kNone;

  TopicDropArgs drop_args_;
  TopicKeepArgs keep_args_;
  TopicRenameArgs rename_args_;

  void configure_drop(CLI::App & app)
  {
    auto * sub = app.add_subcommand(
      "drop",
      "Remove topics from a rosbag, copying every other topic verbatim. Each "
      "<topic> is a literal name or a '*' glob (e.g. /sensing/*); '*' matches any "
      "run of characters, including '/'.");
    sub->add_option("input", drop_args_.input_path, "Input ROS 2 rosbag (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    sub
      ->add_option(
        "-t,--topics", drop_args_.topics,
        "Topic selector(s) to remove. A literal topic name or a '*' glob. Repeat for several.")
      ->required();
    sub->add_option(
      "-o,--output", drop_args_.output_path,
      "Write the result to this new bag instead of rewriting <input> in place.");
    sub->add_flag(
      "-w,--overwrite", drop_args_.overwrite,
      "Replace an existing -o/--output path. Without it, an existing output path stops the run.");
    sub->callback([this]() { selected_ = Subcommand::kDrop; });
  }

  void configure_keep(CLI::App & app)
  {
    auto * sub = app.add_subcommand(
      "keep",
      "Keep only the selected topics, dropping every other topic. Each "
      "<topic> is a literal name or a '*' glob (e.g. /sensing/*); '*' matches any "
      "run of characters, including '/'.");
    sub->add_option("input", keep_args_.input_path, "Input ROS 2 rosbag (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    sub
      ->add_option(
        "-t,--topics", keep_args_.topics,
        "Topic selector(s) to keep. A literal topic name or a '*' glob. Repeat for several.")
      ->required();
    sub->add_option(
      "-o,--output", keep_args_.output_path,
      "Write the result to this new bag instead of rewriting <input> in place.");
    sub->add_flag(
      "-w,--overwrite", keep_args_.overwrite,
      "Replace an existing -o/--output path. Without it, an existing output path stops the run.");
    sub->callback([this]() { selected_ = Subcommand::kKeep; });
  }

  void configure_rename(CLI::App & app)
  {
    auto * sub = app.add_subcommand(
      "rename",
      "Rename one topic in a rosbag, copying every other topic verbatim. <src_topic> and "
      "<dst_topic> are literal topic names (no globs): the topic named <src_topic> is "
      "re-declared as <dst_topic> and its messages move with it.");
    sub->add_option("input", rename_args_.input_path, "Input ROS 2 rosbag (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    sub->add_option("src_topic", rename_args_.src_topic, "Existing topic to rename (literal name).")
      ->required();
    sub->add_option("dst_topic", rename_args_.dst_topic, "New name for the topic (literal name).")
      ->required();
    sub->add_option(
      "-o,--output", rename_args_.output_path,
      "Write the result to this new bag instead of rewriting <input> in place.");
    sub->add_flag(
      "-w,--overwrite", rename_args_.overwrite,
      "Replace an existing -o/--output path. Without it, an existing output path stops the run.");
    sub->callback([this]() { selected_ = Subcommand::kRename; });
  }
};

BAGWIZ_REGISTER_COMMAND(TopicCommand)

}  // namespace bagwiz::commands
