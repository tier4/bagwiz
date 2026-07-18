// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__CAM_INFO_DUMP_HPP_
#define BAGWIZ__COMMANDS__CAM_INFO_DUMP_HPP_

#include <filesystem>
#include <optional>
#include <string>

namespace bagwiz::commands
{

// Parsed arguments for `bagwiz cam-info dump`, which writes one CameraInfo
// topic's calibration out as a standard camera_calibration YAML.
//
// The dump is verbatim: every field is copied from the bag as recorded. p in
// particular is NOT recomputed -- `bagwiz cam-info recompute-p <the dumped
// yaml>` does that, so the two commands compose rather than overlap.
//
// A camera_calibration YAML holds exactly one calibration, so `topic` is a
// single topic rather than a list, and the first message's calibration is the
// one written (a stream whose calibration is not constant is reported).
struct CamInfoDumpArgs
{
  std::filesystem::path input_path;                  // input ROS 2 bag; only ever read
  std::string topic;                                 // one sensor_msgs/msg/CameraInfo topic
  std::optional<std::filesystem::path> output_path;  // empty = write to stdout
  bool overwrite = false;                            // replace an existing -o/--output path
};

// Run the dump. Returns the process exit code: 0 on success, 1 on any error (an
// unreadable bag, a topic that is missing / not a CameraInfo topic / carries no
// messages, or an I/O failure). Kept as a free function in its own translation
// unit so the CamInfoCommand dispatcher in cam_info.cpp stays small; declared
// here so cam_info.cpp can call it.
int run_cam_info_dump(const CamInfoDumpArgs & args);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__CAM_INFO_DUMP_HPP_
