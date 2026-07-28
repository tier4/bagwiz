// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagwiz/commands/cam_info_dump.hpp"
#include "bagwiz/commands/cam_info_recompute_p.hpp"
#include "bagwiz/commands/cam_info_replace.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/core/base/logging.hpp"

#include <string_view>

namespace bagwiz::commands
{

namespace
{
constexpr const char * kLogger = "bagwiz.cmd.cam-info";
}  // namespace

// `bagwiz cam-info` is a command group for sensor_msgs/msg/CameraInfo
// operations. It ships `replace` (swap one or more CameraInfo topics'
// calibration for the values in a single standard ROS camera_calibration YAML
// file), `recompute-p` (derive the projection matrix from the intrinsics it
// belongs to), and `dump` (write a topic's calibration back out to YAML).
// Modeling it as a group keeps each action's options on the action that owns
// them, rather than a flat command accreting every one of them.
class CamInfoCommand : public Command
{
public:
  [[nodiscard]] std::string_view name() const override { return "cam-info"; }
  [[nodiscard]] std::string_view description() const override
  {
    return "Operate on CameraInfo topics";
  }

  void configure(CLI::App & app) override
  {
    app.require_subcommand(1);
    configure_replace(app);
    configure_recompute_p(app);
    configure_dump(app);
  }

  int run() override
  {
    switch (selected_) {
      case Subcommand::kReplace:
        return run_cam_info_replace(replace_args_);
      case Subcommand::kRecomputeP:
        return run_cam_info_recompute_p(recompute_p_args_);
      case Subcommand::kDump:
        return run_cam_info_dump(dump_args_);
      case Subcommand::kNone:
        BAGWIZ_LOG_ERROR(kLogger, "no subcommand selected");
        return 1;
    }
    return 1;
  }

private:
  enum class Subcommand { kNone, kReplace, kRecomputeP, kDump };
  Subcommand selected_ = Subcommand::kNone;

  CamInfoReplaceArgs replace_args_;
  CamInfoRecomputePArgs recompute_p_args_;
  CamInfoDumpArgs dump_args_;

  void configure_replace(CLI::App & app)
  {
    auto * sub = app.add_subcommand(
      "replace",
      "Replace the calibration carried by one or more sensor_msgs/msg/CameraInfo topics with the "
      "values from a single standard ROS camera_calibration YAML file");
    sub
      ->add_option("-i,--input", replace_args_.input_path, "Input ROS 2 rosbag (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    sub
      ->add_option(
        "--yaml", replace_args_.yaml_path,
        "Camera calibration YAML in the camera_calibration / camera_info_manager format")
      ->required()
      ->check(CLI::ExistingFile);
    sub
      ->add_option(
        "-t,--topics", replace_args_.topics,
        "One or more CameraInfo topics to rewrite (each type must be "
        "sensor_msgs/msg/CameraInfo). The same calibration YAML is applied to every listed topic.")
      ->required();
    sub->add_option(
      "--frame-id", replace_args_.frame_id,
      "Override header.frame_id on the rewritten messages. When omitted, each message keeps its "
      "original frame_id.");
    sub->add_option(
      "-o,--output", replace_args_.output_path,
      "Write the result to this new bag instead of rewriting <input> in place.");
    sub->add_flag(
      "-w,--overwrite", replace_args_.overwrite,
      "Replace an existing -o/--output path. Without it, an existing output path stops the run. "
      "Has no effect in in-place mode (when -o is omitted, <input> is replaced atomically by "
      "design).");
    sub->callback([this]() { selected_ = Subcommand::kReplace; });
  }

  void configure_recompute_p(CLI::App & app)
  {
    auto * sub = app.add_subcommand(
      "recompute-p",
      "Recompute the projection matrix (p / projection_matrix) from the intrinsics it belongs to, "
      "in a camera_calibration YAML file or in a bag's sensor_msgs/msg/CameraInfo topics");
    sub
      ->add_option(
        "-i,--input", recompute_p_args_.input_path,
        "Calibration YAML (a .yaml/.yml file in the camera_calibration / camera_info_manager "
        "format) or an input ROS 2 rosbag (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    sub->add_option(
      "-t,--topics", recompute_p_args_.topics,
      "Bag input only: one or more CameraInfo topics whose p to recompute (each type must be "
      "sensor_msgs/msg/CameraInfo). Required when <input> is a bag; rejected when <input> is a "
      "calibration YAML, which carries no topics.");
    sub
      ->add_option(
        "-a,--alpha", recompute_p_args_.alpha,
        "OpenCV free-scaling parameter passed to getOptimalNewCameraMatrix. 0 keeps only valid "
        "pixels (the camera_calibration default); 1 retains every source pixel, leaving black "
        "borders.")
      ->check(CLI::Range(0.0, 1.0))
      ->capture_default_str();
    sub->add_option(
      "-o,--output", recompute_p_args_.output_path,
      "Write the result to this new path instead of rewriting <input> in place.");
    sub->add_flag(
      "-w,--overwrite", recompute_p_args_.overwrite,
      "Replace an existing -o/--output path. Without it, an existing output path stops the run. "
      "Has no effect in in-place mode (when -o is omitted, <input> is replaced atomically by "
      "design).");
    sub->callback([this]() { selected_ = Subcommand::kRecomputeP; });
  }

  void configure_dump(CLI::App & app)
  {
    auto * sub = app.add_subcommand(
      "dump",
      "Write a sensor_msgs/msg/CameraInfo topic's calibration out as a standard ROS "
      "camera_calibration YAML file");
    sub->add_option("-i,--input", dump_args_.input_path, "Input ROS 2 rosbag (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    sub
      ->add_option(
        "-t,--topic", dump_args_.topic,
        "The CameraInfo topic whose calibration to write (its type must be "
        "sensor_msgs/msg/CameraInfo)")
      ->required();
    sub->add_option(
      "-o,--output", dump_args_.output_path, "Write the YAML to this path instead of stdout.");
    sub->add_flag(
      "-w,--overwrite", dump_args_.overwrite,
      "Replace an existing -o/--output path. Without it, an existing output path stops the run. "
      "Has no effect without -o (there is nothing to overwrite when writing to stdout).");
    sub->callback([this]() { selected_ = Subcommand::kDump; });
  }
};

BAGWIZ_REGISTER_COMMAND(CamInfoCommand)

}  // namespace bagwiz::commands
