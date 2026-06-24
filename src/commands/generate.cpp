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
#include "bagwiz/core/pointcloud/color_scheme.hpp"
#include "bagwiz/core/pointcloud/property.hpp"

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
    auto * sub =
      app.add_subcommand("video", "Render an image topic from a rosbag to a video file.");
    sub->add_option("<input>", video_args_.input_path, "Input ROS 2 rosbag (file or directory).")
      ->required()
      ->check(CLI::ExistingPath);
    sub
      ->add_option(
        "<img_topic>", video_args_.topic,
        "Image topic to render. Supported types: sensor_msgs/msg/Image (bgr8, rgb8) and "
        "sensor_msgs/msg/CompressedImage (JPEG/PNG).")
      ->required();
    sub
      ->add_option(
        "<output>", video_args_.output_path,
        "Output video path. Extension selects container/codec: .mp4/.mkv/.mov -> H.264, "
        ".avi -> MJPEG.")
      ->required();
    sub->add_flag(
      "-w,--overwrite", video_args_.overwrite,
      "Replace an existing <output>. Without it, an existing output path stops the run.");
    sub
      ->add_option(
        "--cam-info", video_args_.camera_info_topic,
        "CameraInfo topic for --undistort and --pcd. When omitted, bagwiz derives it from "
        "<img_topic>: /image_raw/compressed, /image_rect_color, and /image_rect_color/compressed "
        "map their prefix to /camera_info.")
      ->check([](const std::string & topic) {
        if (topic.empty()) {
          return std::string{"cam-info topic must not be empty"};
        }
        return std::string{};
      });
    sub->add_flag(
      "--undistort", video_args_.undistort,
      "Apply distortion correction to each frame using the resolved CameraInfo. "
      "Requires a camera-info topic; use --cam-info if auto-resolution fails.");
    sub
      ->add_option(
        "--resize", video_args_.resize_scale,
        "Scale output width and height by this factor while preserving aspect ratio. "
        "1.0 keeps the original size, 0.5 halves both dimensions, 2.0 doubles them.")
      ->default_val(1.0f)
      ->check(CLI::Range(0.01f, 10.0f));
    sub
      ->add_option(
        "--pcd", video_args_.pointcloud_topics,
        "PointCloud2 topic(s) to project onto each frame. Repeatable; every listed topic is "
        "projected into the camera frame and drawn with the same field, color scheme, point size, "
        "and alpha. Implies distortion correction and requires a CameraInfo topic and a TF chain "
        "from each cloud frame to the camera frame.")
      ->check([](const std::string & topic) {
        if (topic.empty()) {
          return std::string{"pcd topic must not be empty"};
        }
        return std::string{};
      })
      ->expected(-1);
    const std::map<std::string, core::pointcloud::PointCloudProperty> property_map = {
      {"x", core::pointcloud::PointCloudProperty::kX},
      {"y", core::pointcloud::PointCloudProperty::kY},
      {"z", core::pointcloud::PointCloudProperty::kZ},
      {"distance", core::pointcloud::PointCloudProperty::kDistance},
      {"intensity", core::pointcloud::PointCloudProperty::kIntensity}};
    sub
      ->add_option(
        "--field", video_args_.property,
        "Point-cloud field used for coloring: x, y, z, distance, intensity.")
      ->transform(CLI::CheckedTransformer{property_map})
      ->default_val(core::pointcloud::PointCloudProperty::kDistance);
    sub
      ->add_option(
        "--min", video_args_.property_min, "Manual minimum value for field normalization.")
      ->capture_default_str();
    sub
      ->add_option(
        "--max", video_args_.property_max, "Manual maximum value for field normalization.")
      ->capture_default_str();
    const std::map<std::string, core::pointcloud::ColorScheme> scheme_map = {
      {"viridis", core::pointcloud::ColorScheme::kViridis},
      {"turbo", core::pointcloud::ColorScheme::kTurbo},
      {"jet", core::pointcloud::ColorScheme::kJet},
      {"plasma", core::pointcloud::ColorScheme::kPlasma},
      {"inferno", core::pointcloud::ColorScheme::kInferno},
      {"magma", core::pointcloud::ColorScheme::kMagma},
      {"rainbow", core::pointcloud::ColorScheme::kRainbow}};
    sub->add_option("--scheme", video_args_.colorscheme, "Color scheme for point coloring.")
      ->transform(CLI::CheckedTransformer{scheme_map})
      ->default_val(core::pointcloud::ColorScheme::kViridis);
    sub->add_option("--point-size", video_args_.point_size, "Diameter of drawn points in pixels.")
      ->default_val(2U)
      ->check(CLI::Range(1U, 64U));
    sub->add_option("--alpha", video_args_.alpha, "Point overlay opacity.")
      ->default_val(1.0f)
      ->check(CLI::Range(0.0f, 1.0f));
    sub->footer(
      "Frames stream straight to the encoder (no large temp files); the output is written\n"
      "atomically and a failed run leaves no partial file behind.");
    sub->callback([this]() { selected_ = Subcommand::kVideo; });
  }
};

BAGWIZ_REGISTER_COMMAND(GenerateCommand)

}  // namespace bagwiz::commands
