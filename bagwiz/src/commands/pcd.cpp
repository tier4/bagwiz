// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/commands/pcd_concat.hpp"
#include "bagwiz/commands/pcd_undistort.hpp"
#include "bagwiz/core/base/logging.hpp"

#include <string_view>

namespace bagwiz::commands
{

namespace
{
constexpr const char * kLogger = "bagwiz.cmd.pcd";
}  // namespace

// `bagwiz pcd` is a command group for PointCloud2 topic processing.
//   concat    - merge multiple PointCloud2 topics into one new topic (static TF +
//               first-topic-driven time sync)
//   undistort - motion-deskew PointCloud2 topic(s) using external pose topic + tf_static
class PcdCommand : public Command
{
public:
  [[nodiscard]] std::string_view name() const override { return "pcd"; }
  [[nodiscard]] std::string_view description() const override
  {
    return "PointCloud2 topic processing";
  }

  void configure(CLI::App & app) override
  {
    app.require_subcommand(1);
    configure_concat(app);
    configure_undistort(app);
  }

  int run() override
  {
    switch (selected_) {
      case Subcommand::kConcat:
        return run_pcd_concat(concat_args_);
      case Subcommand::kUndistort:
        return run_pcd_undistort(undistort_args_);
      case Subcommand::kNone:
        BAGWIZ_LOG_ERROR(kLogger, "no subcommand selected");
        return 1;
    }
    return 1;
  }

private:
  enum class Subcommand { kNone, kConcat, kUndistort };
  Subcommand selected_ = Subcommand::kNone;
  PcdConcatArgs concat_args_;
  PcdUndistortArgs undistort_args_;

  void configure_concat(CLI::App & app)
  {
    auto * sub = app.add_subcommand(
      "concat",
      "Merge multiple PointCloud2 topics into one new topic. Each topic is rigidly "
      "transformed into --frame using the bag's static TF, and messages are matched against "
      "the first --pcd topic within --tolerance (nearest-in-time).");
    sub->add_option("input", concat_args_.input_path, "Bag path (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    sub
      ->add_option(
        "output_topic", concat_args_.output_topic, "Name of the new concatenated PointCloud2 topic")
      ->required();
    sub
      ->add_option(
        "--pcd", concat_args_.pcd_topics,
        "PointCloud2 topics to concatenate (2 or more). Concatenation order follows this list.")
      ->required()
      ->expected(-1);
    sub->add_option(
      "--frame", concat_args_.frame,
      "Target frame all clouds are transformed into. Default: base_link. Required when the "
      "default is not reachable from every --pcd frame via the bag's static TF.");
    sub->add_option(
      "-o,--output", concat_args_.output_path,
      "Output bag path. When omitted, the input bag is rewritten in place (atomic tmp swap).");
    sub->add_option(
      "--tolerance", concat_args_.tolerance,
      "Nearest-match tolerance for pairing the other topics to the first --pcd topic. "
      "Takes an optional unit ns/us/ms/s (no unit = ms), e.g. 50ms. "
      "Default: half the first topic's median period.");
    sub->add_option(
      "--stamp-offset", concat_args_.stamp_offsets,
      "Per-topic matching offset as topic=value, added to header.stamp for MATCHING ONLY "
      "(the real stamp and per-point times are never rewritten). Value takes an optional unit "
      "ns/us/ms/s (no unit = ms), e.g. '/lidar/left/points=50ms'. Repeatable.");
    sub->add_flag(
      "--drop-inputs", concat_args_.drop_inputs,
      "Drop the source --pcd topics from the output (default: keep them).");
    sub->add_flag(
      "--force", concat_args_.force,
      "Proceed even if <output_topic> already exists in the bag (replaces that topic).");
    sub->add_flag(
      "-w,--overwrite", concat_args_.overwrite, "Overwrite an existing -o/--output path.");
    sub->callback([this]() { selected_ = Subcommand::kConcat; });
  }

  void configure_undistort(CLI::App & app)
  {
    auto * sub = app.add_subcommand(
      "undistort", "Motion-deskew PointCloud2 topic(s) using an external pose topic + tf_static.");
    sub->add_option("input", undistort_args_.input_path, "Input bag (file or directory).")
      ->required()
      ->check(CLI::ExistingPath);
    sub
      ->add_option(
        "pose_topic", undistort_args_.pose_topic,
        "Self-position topic (TFMessage / Odometry / PoseStamped / PoseWithCovarianceStamped).")
      ->required();
    sub
      ->add_option(
        "--pcd", undistort_args_.pcd_topics, "PointCloud2 topic(s) to deskew (repeatable).")
      ->required()
      ->expected(-1);
    sub->add_option("--ref", undistort_args_.ref_frame, "Reference frame (default: map).");
    sub->add_option("--of", undistort_args_.of_frame, "Tracked body frame (default: base_link).");
    sub->add_option(
      "-o,--output", undistort_args_.output_path,
      "Output bag. Omitted => rewrite <input> in place.");
    sub->add_flag(
      "-w,--overwrite", undistort_args_.overwrite, "Replace -o output if it already exists.");
    sub
      ->add_option(
        "-j,--threads", undistort_args_.threads,
        "Number of worker threads for deskew (default: 8; 0 = hardware concurrency, "
        "1 = sync). Values above hardware concurrency are capped.")
      ->check(CLI::Range(0, 256));
    sub->callback([this]() { selected_ = Subcommand::kUndistort; });
  }
};

BAGWIZ_REGISTER_COMMAND(PcdCommand)

}  // namespace bagwiz::commands
