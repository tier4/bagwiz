// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__TF_WALK_HPP_
#define BAGWIZ__COMMANDS__TF_WALK_HPP_

#include <filesystem>
#include <string>

namespace bagwiz::commands
{

// Implements `bagwiz tf walk <input> --of <of> --ref <ref>`: merge every
// tf2_msgs/msg/TFMessage topic in the bag into one tf2 buffer and step through
// the distinct times at which the merged TF changed, rendering <of>'s pose in
// <ref> at each in an interactive pager (the same UX as `bagwiz walk`). The
// walk does not classify transforms as static vs dynamic.
//
// Returns the process exit code: 0 on a clean quit, 1 on a setup error (no
// TTY, bag could not be opened, no TF topic, no decodable transforms). Kept as
// a free function in its own translation unit so the TfCommand dispatcher in
// tf.cpp stays small; declared here so tf.cpp can call it.
int run_tf_walk(
  const std::filesystem::path & input_path, const std::string & of_frame,
  const std::string & ref_frame);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__TF_WALK_HPP_
