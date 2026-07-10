// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__PCD_UNDISTORT_HPP_
#define BAGWIZ__COMMANDS__PCD_UNDISTORT_HPP_

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace bagwiz::commands
{

// Parsed arguments for `bagwiz pcd undistort`. Deskews one or more PointCloud2
// topics using a pose topic as the motion source, resolved through the bag's
// static TF (no dynamic /tf; SLAM-free).
struct PcdUndistortArgs
{
  std::filesystem::path input_path;                  // <input> bag
  std::string pose_topic;                            // <pose_topic> positional (motion source)
  std::vector<std::string> pcd_topics;               // --pcd (>=1)
  std::optional<std::string> from_frame;             // --from; empty => "map"
  std::optional<std::string> to_frame;               // --to;   empty => "base_link"
  std::optional<std::filesystem::path> output_path;  // -o; empty => in-place
  bool overwrite = false;                            // -w
  std::optional<int> threads;  // -t,--threads; 0/omit => hardware concurrency, 1 => sync
};

// Execute `bagwiz pcd undistort`. Returns a process exit code.
int run_pcd_undistort(const PcdUndistortArgs & args);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__PCD_UNDISTORT_HPP_
