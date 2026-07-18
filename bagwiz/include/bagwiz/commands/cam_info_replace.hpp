// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__CAM_INFO_REPLACE_HPP_
#define BAGWIZ__COMMANDS__CAM_INFO_REPLACE_HPP_

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace bagwiz::commands
{

// Parsed arguments for `bagwiz cam-info replace`. The command rewrites one or
// more sensor_msgs/msg/CameraInfo topics so that every message carries the
// calibration from a single standard ROS camera_calibration YAML file — the
// same YAML is applied to every listed topic. Each message's header timestamp
// (and `frame_id` unless `--frame-id` is given), binning_x/y, and roi are
// preserved; only the calibration fields (height, width, distortion_model, d,
// k, r, p) are overwritten. Every other topic is copied verbatim.
struct CamInfoReplaceArgs
{
  std::filesystem::path input_path;                  // bag to rewrite (also the in-place target)
  std::filesystem::path yaml_path;                   // source camera_calibration YAML
  std::vector<std::string> topics;                   // CameraInfo topic(s) to rewrite (>= 1)
  std::optional<std::string> frame_id;               // override header.frame_id when set
  std::optional<std::filesystem::path> output_path;  // empty = in-place rewrite
  bool overwrite = false;                            // replace an existing -o/--output path
};

// Run the replacement. Returns the process exit code: 0 on success, 1 on any
// error (bag open failure, topic missing or of the wrong type, YAML parse
// failure, deserialize/serialize failure, or I/O error). Kept as a free
// function in its own translation unit so the CamInfoCommand dispatcher in
// cam_info.cpp stays small; declared here so cam_info.cpp can call it.
int run_cam_info_replace(const CamInfoReplaceArgs & args);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__CAM_INFO_REPLACE_HPP_
