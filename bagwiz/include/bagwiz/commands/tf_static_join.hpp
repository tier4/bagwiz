// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BAGWIZ__COMMANDS__TF_STATIC_JOIN_HPP_
#define BAGWIZ__COMMANDS__TF_STATIC_JOIN_HPP_

#include <filesystem>
#include <optional>
#include <string>

namespace bagwiz::commands
{

// The topic `tf static join` writes when -t/--topic is omitted. It is the name a
// static transform broadcaster publishes under, and the one every bagwiz static-TF
// reader recognises.
inline constexpr const char * kDefaultStaticTfTopic = "/tf_static";

// Implements `bagwiz tf static join -i <input> --yaml <file> [-t <topic>]
// [-o <output>] [--force] [-w|--overwrite]`: read a static-transform publisher
// config — the nested parent -> child -> {x,y,z,roll,pitch,yaw} YAML that
// `bagwiz tf static dump` writes, with rotations as RPY in radians — and embed it
// into the bag as one latched `tf2_msgs/msg/TFMessage` on `topic`. The inverse of
// `tf static dump`; see core::parse_static_tf_tree_yaml for the accepted schema
// and core::rpy_to_quaternion for the rotation conversion.
//
// Nesting may go arbitrarily deep, as it may for the reference publisher: a level
// that holds no transform of its own is a grouping heading rather than a frame.
// Since an author may have meant such a level as a chain link, the keys that
// turned out to parent nothing are warned about (parse's `grouping_frames`).
//
// The message is stamped at `<input>`'s earliest message time — both its receive
// time and every transform's header.stamp — which places the latched static TF at
// the very start of the timeline, where a static transform is expected to already
// hold. It is written ahead of the copied messages so its storage order agrees
// with its timestamp (see inject_static_tf_pass).
//
// `force` permits replacing a `topic` that already carries messages in `<input>`;
// without it such a collision aborts the run. A collision with a topic of a
// different message type is always an error. `overwrite` separately permits
// replacing an existing `-o`/`--output` path, matching `bagwiz traj join`.
//
// When `output_path` is empty, `<input>` is rewritten in place via an atomic
// tmp-swap that preserves its storage format and layout; when it is set,
// `<input>` is left untouched and the result is written there.
//
// Returns the process exit code: 0 on success, 1 on any error (the YAML could not
// be read or is invalid, a bag could not be opened, an unresolved topic/type
// conflict, a serialize failure, or an I/O error).
int run_tf_static_join(
  const std::filesystem::path & input_path, const std::filesystem::path & yaml_path,
  const std::string & topic, const std::optional<std::filesystem::path> & output_path, bool force,
  bool overwrite);

}  // namespace bagwiz::commands

#endif  // BAGWIZ__COMMANDS__TF_STATIC_JOIN_HPP_
