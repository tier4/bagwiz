// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagwiz/commands/cam_info_replace.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/core/logging.hpp"

#include <string_view>

namespace bagwiz::commands
{

namespace
{
constexpr const char * kLogger = "bagwiz.cmd.cam-info";
}  // namespace

// `bagwiz cam-info` is a command group for sensor_msgs/msg/CameraInfo
// operations. It ships `replace` (swap a CameraInfo topic's calibration for the
// one in a standard ROS camera_calibration YAML file). Modeling it as a group
// leaves room for further actions (e.g. a future `dump` that exports a topic's
// calibration back out to YAML) without a flat command accreting every option.
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
  }

  int run() override
  {
    switch (selected_) {
      case Subcommand::kReplace:
        return run_cam_info_replace(replace_args_);
      case Subcommand::kNone:
        BAGWIZ_LOG_ERROR(kLogger, "no subcommand selected");
        return 1;
    }
    return 1;
  }

private:
  enum class Subcommand { kNone, kReplace };
  Subcommand selected_ = Subcommand::kNone;

  CamInfoReplaceArgs replace_args_;

  void configure_replace(CLI::App & app)
  {
    auto * sub = app.add_subcommand(
      "replace",
      "Replace the calibration carried by a sensor_msgs/msg/CameraInfo topic with the values from "
      "a standard ROS camera_calibration YAML file");
    sub->add_option("input", replace_args_.input_path, "Input ROS 2 rosbag (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    sub
      ->add_option(
        "calib_yaml", replace_args_.yaml_path,
        "Camera calibration YAML in the camera_calibration / camera_info_manager format "
        "(image_width, image_height, camera_matrix, distortion_model, distortion_coefficients, "
        "rectification_matrix, projection_matrix)")
      ->required()
      ->check(CLI::ExistingFile);
    sub
      ->add_option(
        "topic", replace_args_.topic,
        "CameraInfo topic to rewrite (its type must be sensor_msgs/msg/CameraInfo)")
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
    sub->footer(
      "Only the named CameraInfo topic is rewritten; every other topic is copied verbatim. For "
      "each message on the topic, the calibration fields (height, width, distortion_model, d, k, "
      "r, p) are taken from the YAML while the original header timestamp, frame_id (unless "
      "--frame-id is given), binning_x/y, and roi are preserved. The YAML's camera_name, if "
      "present, is informational only and ignored.");
    sub->callback([this]() { selected_ = Subcommand::kReplace; });
  }
};

BAGWIZ_REGISTER_COMMAND(CamInfoCommand)

}  // namespace bagwiz::commands
