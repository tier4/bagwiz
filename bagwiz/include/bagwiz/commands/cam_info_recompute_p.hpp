// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__CAM_INFO_RECOMPUTE_P_HPP_
#define BAGWIZ__COMMANDS__CAM_INFO_RECOMPUTE_P_HPP_

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace bagwiz::commands
{

// Parsed arguments for `bagwiz cam-info recompute-p`, which recomputes a
// projection matrix from the intrinsics it belongs to:
//
//   p = [ cv::getOptimalNewCameraMatrix(k, d, (width, height), alpha) | 0 ]
//
// `input_path` selects the mode, and the result always has the same shape as it:
// a `.yaml`/`.yml` file is a camera_calibration YAML, so its projection_matrix
// block is recomputed and the file re-emitted, and `topics` must be empty (a
// YAML carries no topics). Anything else is a ROS 2 bag, so every message on
// each listed CameraInfo topic has its p recomputed from that same message's own
// k/d/width/height, and `topics` must name at least one.
//
// `output_path`'s extension chooses nothing; it only says where the result goes.
// For a bag it does pick the storage format (.mcap/.db3 convert, anything else
// inherits `input_path`'s) via io::create_options_inheriting_format(). To pull a
// bag's calibration out as a YAML, use `bagwiz cam-info dump`
// (commands/cam_info_dump.hpp) instead.
//
// `topics` is required-for-a-bag / rejected-for-a-YAML, which CLI11 cannot
// express as a parse-time constraint, so run_cam_info_recompute_p() validates it
// once the mode is known and reports either violation as an error.
struct CamInfoRecomputePArgs
{
  std::filesystem::path input_path;                  // calibration YAML or bag (in-place target)
  std::vector<std::string> topics;                   // -t/--topics; bag mode only, >= 1
  double alpha = 0.0;                                // OpenCV free-scaling parameter, [0, 1]
  std::optional<std::filesystem::path> output_path;  // empty = rewrite <input> in place
  bool overwrite = false;                            // replace an existing -o/--output path
};

// Run the recomputation. Returns the process exit code: 0 on success, 1 on any
// error (unreadable YAML or bag, a topic that is missing or not a CameraInfo
// topic, a calibration whose p cannot be recomputed from k -- stereo-rectified,
// carrying a baseline, or fisheye -- or an I/O failure). Kept as a free function
// in its own translation unit so the CamInfoCommand dispatcher in cam_info.cpp
// stays small; declared here so cam_info.cpp can call it.
int run_cam_info_recompute_p(const CamInfoRecomputePArgs & args);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__CAM_INFO_RECOMPUTE_P_HPP_
