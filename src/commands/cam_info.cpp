// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagwiz/commands/cam_info_recompute_p.hpp"
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
// operations. It ships `replace` (swap one or more CameraInfo topics'
// calibration for the values in a single standard ROS camera_calibration YAML
// file) and `recompute-p` (derive the projection matrix from the intrinsics it
// belongs to). Modeling it as a group leaves room for further actions (e.g. a
// future `dump` that exports a topic's calibration back out to YAML) without a
// flat command accreting every option.
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
  }

  int run() override
  {
    switch (selected_) {
      case Subcommand::kReplace:
        return run_cam_info_replace(replace_args_);
      case Subcommand::kRecomputeP:
        return run_cam_info_recompute_p(recompute_p_args_);
      case Subcommand::kNone:
        BAGWIZ_LOG_ERROR(kLogger, "no subcommand selected");
        return 1;
    }
    return 1;
  }

private:
  enum class Subcommand { kNone, kReplace, kRecomputeP };
  Subcommand selected_ = Subcommand::kNone;

  CamInfoReplaceArgs replace_args_;
  CamInfoRecomputePArgs recompute_p_args_;

  void configure_replace(CLI::App & app)
  {
    auto * sub = app.add_subcommand(
      "replace",
      "Replace the calibration carried by one or more sensor_msgs/msg/CameraInfo topics with the "
      "values from a single standard ROS camera_calibration YAML file");
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
        "topics", replace_args_.topics,
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
    sub->footer(
      "Only the named CameraInfo topics are rewritten; every other topic is copied verbatim. For "
      "each message on those topics, the calibration fields (height, width, distortion_model, d, "
      "k, r, p) are taken from the YAML while the original header timestamp, frame_id (unless "
      "--frame-id is given), binning_x/y, and roi are preserved. The YAML's camera_name, if "
      "present, is informational only and ignored.");
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
        "input", recompute_p_args_.input_path,
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
    sub->footer(
      "<input> says where the calibration comes from: a .yaml/.yml file is a camera_calibration "
      "YAML (--topics does not apply), anything else is a ROS 2 bag (--topics is required). For a "
      "bag, -o then says what to produce -- a .yaml/.yml output means 'give me the calibration' "
      "rather than 'rewrite the bag':\n"
      "\n"
      "  <input>  -o           result\n"
      "  YAML     (none)/YAML  the YAML is rewritten\n"
      "  bag      (none)/bag   the bag is rewritten\n"
      "  bag      YAML         the topic's calibration is written out; the bag is untouched\n"
      "\n"
      "Exporting a bag to YAML needs exactly one --topics entry, since a camera_calibration YAML "
      "holds a single calibration. The first message's calibration is used, and a topic whose "
      "calibration is not constant across the bag is reported.\n"
      "\n"
      "p is recomputed as [getOptimalNewCameraMatrix(k, d, (width, height), alpha) | 0], so k, d, "
      "and the image size are the inputs -- everything else in the file (or message) is preserved. "
      "For a bag, only the named topics are touched and each message's p is recomputed from that "
      "same message's own k/d/width/height; header, binning, roi, and every other topic are copied "
      "verbatim.\n"
      "\n"
      "Supported distortion_model values: 'plumb_bob' (5 coefficients), 'rational_polynomial' (8), "
      "and an empty model or 'none' (declares no distortion, so p is [k | 0]). Any other model -- "
      "including the fisheye family ('equidistant', 'fisheye'), which needs "
      "cv::fisheye::estimateNewCameraMatrixForUndistortRectify and a `balance` instead of an alpha "
      "-- is an error, checked before d so it is refused even with all-zero coefficients.\n"
      "\n"
      "Also refused where recomputing from k would be wrong rather than imprecise: a "
      "stereo-rectified camera (non-identity r), or a p carrying a stereo baseline (non-zero "
      "p[3]/p[7]). Nothing is written when a run is refused.\n"
      "\n"
      "Expect a sub-pixel difference from a p an older camera_calibration wrote, rather than an "
      "identical value: getOptimalNewCameraMatrix shifts slightly between OpenCV versions. The "
      "run logs how far p moved so a small change is legible as version drift.\n"
      "\n"
      "Note a YAML rewrite is a re-emit: values are preserved but comments and formatting are "
      "normalized. Use -o to keep the original file untouched.");
    sub->callback([this]() { selected_ = Subcommand::kRecomputeP; });
  }
};

BAGWIZ_REGISTER_COMMAND(CamInfoCommand)

}  // namespace bagwiz::commands
