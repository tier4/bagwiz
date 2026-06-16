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
    sub->add_option("input", video_args_.input_path, "Input ROS 2 rosbag (file or directory)")
      ->required()
      ->check(CLI::ExistingPath);
    sub->add_option("topic", video_args_.topic, "Image topic to render.")->required();
    sub
      ->add_option(
        "output", video_args_.output_path,
        "Output video path. Its extension selects the output format: "
        ".mp4/.mkv/.mov -> H.264, .avi -> MJPEG.")
      ->required();
    sub->add_flag(
      "-w,--overwrite", video_args_.overwrite,
      "Replace an existing <output>. Without it, an existing output path stops the run.");
    sub->footer(
      "Supported topic types: sensor_msgs/msg/Image (bgr8, rgb8) and "
      "sensor_msgs/msg/CompressedImage (JPEG/PNG, decoded to BGR before encoding).\n"
      "Frames stream straight to the encoder (no large temp files); the output is written\n"
      "atomically and a failed run leaves no partial file behind.");
    sub->callback([this]() { selected_ = Subcommand::kVideo; });
  }
};

BAGWIZ_REGISTER_COMMAND(GenerateCommand)

}  // namespace bagwiz::commands
