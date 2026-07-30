// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__TF_STATIC_DUMP_HPP_
#define BAGWIZ__COMMANDS__TF_STATIC_DUMP_HPP_

#include <filesystem>
#include <optional>

namespace bagwiz::commands
{

// Implements `bagwiz tf static dump -i <input> [-o <output>] [-w|--overwrite]`:
// write the bag's static TF tree (topics whose name ends with "tf_static", type
// tf2_msgs/msg/TFMessage) as the nested parent -> child -> {x,y,z,roll,pitch,
// yaw} YAML that static-transform publisher configs use. Rotations are RPY in
// radians in tf2's fixed-axis convention, so a consumer's
// tf2::Quaternion::setRPY reproduces the bag's quaternion; header.stamp has no
// place in the schema and is dropped. See core::emit_static_tf_tree_yaml.
//
// Only the FIRST message of each static topic is read (see
// core::StaticTfRead::kFirstMessagePerTopic): static TF is latched, so that
// message already holds the whole tree and the rest of the bag is skipped.
//
// Every static topic in the bag is merged into one tree, because the schema has
// no topic dimension. Two topics declaring the same child with DIFFERENT parents
// is a contradiction the dump cannot represent and aborts the run, matching
// `bagwiz tf tree` and `bagwiz tf static calc`.
//
// When `output_path` is empty the YAML goes to stdout, so `bagwiz tf static dump
// -i <bag> > tf_static.yaml` is pipe-clean (every diagnostic goes to stderr).
// When it is set, the YAML is written there atomically; `overwrite` permits
// replacing an existing path, and without it a pre-existing path aborts the run.
//
// Returns the process exit code: 0 on success, 1 on any error (bag could not be
// opened, it has no static TF carrying transforms, a decode failure, two topics
// contradicting each other, an existing output without -w/--overwrite, or an
// I/O error). Kept as a free function in its own translation unit so the
// TfCommand dispatcher in tf.cpp stays small; declared here so tf.cpp can call
// it.
int run_tf_static_dump(
  const std::filesystem::path & input_path,
  const std::optional<std::filesystem::path> & output_path, bool overwrite);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__TF_STATIC_DUMP_HPP_
