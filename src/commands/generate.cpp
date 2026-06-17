// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "CLI/CLI.hpp"
#include "bagwiz/commands/command.hpp"
#include "bagwiz/commands/generate_video.hpp"
#include "bagwiz/core/logging.hpp"

#include <map>
#include <string>
#include <string_view>

namespace bagwiz::commands
{

namespace
{
constexpr const char * kLogger = "bagwiz.cmd.generate";
}  // namespace

// `bagwiz generate` is a command group for producing non-rosbag *media* from a
// rosbag (rosbag -> media, not rosbag -> rosbag). Its first subcommand `video`
// renders an image topic to a video file; the group leaves room for further
// media generators (image sequences, GIFs, ...).
class GenerateCommand : public Command
{
public:
  [[nodiscard]] std::string_view name() const override { return "generate"; }
  [[nodiscard]] std::string_view description() const override
  {
    return "Generate non-rosbag media from a rosbag";
  }

  void configure(CLI::App & app) override
  {
    app.require_subcommand(1);
    configure_video(app);
  }

  int run() override
  {
    switch (selected_) {
      case Subcommand::kVideo:
        return run_generate_video(video_args_);
      case Subcommand::kNone:
        BAGWIZ_LOG_ERROR(kLogger, "no subcommand selected");
        return 1;
    }
    return 1;
  }

private:
  enum class Subcommand { kNone, kVideo };
  Subcommand selected_ = Subcommand::kNone;
  GenerateVideoArgs video_args_;

  void configure_video(CLI::App & app)
  {
    auto * sub = app.add_subcommand(
      "video",
      "Render an image topic to a video file. <output>'s extension picks the container/codec "
      "(.mp4/.mkv/.mov -> H.264, .avi -> MJPEG); the frame rate is derived from the message "
      "timestamps.");
    sub->add_option("<input>", video_args_.input_path, "Input ROS 2 rosbag (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    sub->add_option("<img_topic>", video_args_.topic, "Image topic to render.")->required();
    sub
      ->add_option(
        "<output>", video_args_.output_path,
        "Output video path. Its extension selects the output format: "
        ".mp4/.mkv/.mov -> H.264, .avi -> MJPEG. If mpv fails with H.264, try VLC "
        "or run mpv --hwdec=no; .avi (MJPEG) lowers quality.")
      ->required();
    sub
      ->add_option(
        "<pcd_topic>", video_args_.pointcloud_topic,
        "Optional PointCloud2 topic to project onto the video. When omitted, generate video "
        "behaves as before and produces a plain video without point-cloud overlay.")
      ->required(false);
    sub
      ->add_option(
        "<cam_info_topic>", video_args_.camera_info_topic,
        "Optional CameraInfo topic. When omitted, bagwiz derives it from "
        "<img_topic>: image topics ending in /image_raw/compressed, /image_rect_color, or "
        "/image_rect_color/compressed resolve to the sibling /camera_info topic.")
      ->required(false);
    sub->add_flag(
      "-w,--overwrite", video_args_.overwrite,
      "Replace an existing <output>. Without it, an existing output path stops the run.");
    sub->add_flag(
      "--undistort", video_args_.undistort,
      "Apply distortion correction to each frame using the resolved CameraInfo. "
      "When this flag is set, a cam-info topic is required; use <cam_info_topic> if "
      "auto-resolution from the image topic fails.");
    const std::map<std::string, PointCloudProperty> property_map = {
      {"x", PointCloudProperty::kX},
      {"y", PointCloudProperty::kY},
      {"z", PointCloudProperty::kZ},
      {"distance", PointCloudProperty::kDistance},
      {"intensity", PointCloudProperty::kIntensity}};
    sub
      ->add_option(
        "--field", video_args_.property,
        "Point field used for coloring: x, y, z, distance, intensity.")
      ->transform(CLI::CheckedTransformer{property_map})
      ->default_val(PointCloudProperty::kDistance);
    sub
      ->add_option(
        "--min", video_args_.property_min, "Manual minimum value for field normalization.")
      ->capture_default_str();
    sub
      ->add_option(
        "--max", video_args_.property_max, "Manual maximum value for field normalization.")
      ->capture_default_str();
    const std::map<std::string, ColorScheme> scheme_map = {
      {"viridis", ColorScheme::kViridis}, {"turbo", ColorScheme::kTurbo},
      {"jet", ColorScheme::kJet},         {"plasma", ColorScheme::kPlasma},
      {"inferno", ColorScheme::kInferno}, {"magma", ColorScheme::kMagma},
      {"rainbow", ColorScheme::kRainbow}};
    sub->add_option("--scheme", video_args_.colorscheme, "Color scheme for point coloring.")
      ->transform(CLI::CheckedTransformer{scheme_map})
      ->default_val(ColorScheme::kViridis);
    sub->add_option("--point-size", video_args_.point_size, "Diameter of drawn points in pixels.")
      ->default_val(2U)
      ->check(CLI::Range(1U, 64U));
    sub->add_option("--alpha", video_args_.alpha, "Point overlay opacity.")
      ->default_val(1.0f)
      ->check(CLI::Range(0.0f, 1.0f));
    sub->footer(
      "Supported topic types: sensor_msgs/msg/Image (bgr8, rgb8) and "
      "sensor_msgs/msg/CompressedImage (JPEG/PNG, decoded to BGR before encoding).\n"
      "Frames stream straight to the encoder (no large temp files); the output is written\n"
      "atomically and a failed run leaves no partial file behind.\n"
      "Backward compatibility: the original <input> <img_topic> <output> signature is "
      "preserved by keeping <output> before the optional [<pcd_topic>] and "
      "[<cam_info_topic>] positionals. Point-cloud overlays are controlled with --field, "
      "--min, --max, --scheme, --point-size, and --alpha.\n"
      "CameraInfo auto-resolution: /image_raw/compressed, /image_rect_color, and "
      "/image_rect_color/compressed map their prefix to /camera_info.\n"
      "Playback note: H.264 outputs (.mp4/.mkv/.mov) can crash mpv's hardware decoder; "
      "try VLC or run mpv --hwdec=no. Use .avi (MJPEG) only if lower quality is acceptable.");
    sub->callback([this]() { selected_ = Subcommand::kVideo; });
  }
};

BAGWIZ_REGISTER_COMMAND(GenerateCommand)

}  // namespace bagwiz::commands
