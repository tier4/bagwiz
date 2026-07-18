// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__TF_STATIC_CP_HPP_
#define BAGWIZ__COMMANDS__TF_STATIC_CP_HPP_

#include <filesystem>
#include <optional>

namespace bagwiz::commands
{

// Implements `bagwiz tf static cp <src> <dst> [-o <output>] [-w|--overwrite]`:
// copy every static TF topic (name ends with "tf_static", type
// tf2_msgs/msg/TFMessage) from <src> into <dst>, preserving each topic's
// original name. Each copied topic is written as a single TFMessage stamped at
// <dst>'s start time (both the message receive-time and every header.stamp).
//
// When `output_path` is empty, <dst> is rewritten in place via an atomic
// tmp-swap, preserving its storage format and layout. When it is set, <dst> is
// left untouched and the result (<dst>'s messages plus the copied static TF) is
// written to that path.
//
// `overwrite` permits clobbering of either conflict: a pre-existing -o output
// path is replaced, and a static topic in <dst> whose name collides with one
// being copied has its existing messages replaced. Without it, either conflict
// aborts with an explanatory error.
//
// Returns the process exit code: 0 on success, 1 on any error (bag could not be
// opened, <src> has no static TF, decode/serialize failure, an unresolved
// topic/type conflict, or I/O error). Kept as a free function in its own
// translation unit so the TfCommand dispatcher in tf.cpp stays small; declared
// here so tf.cpp can call it.
int run_tf_static_cp(
  const std::filesystem::path & src_path, const std::filesystem::path & dst_path,
  const std::optional<std::filesystem::path> & output_path, bool overwrite);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__TF_STATIC_CP_HPP_
