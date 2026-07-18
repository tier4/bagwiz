// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef COMMANDS__CAM_INFO_COMMON_HPP_
#define COMMANDS__CAM_INFO_COMMON_HPP_

#include "bagwiz/core/pipeline/rewrite_backend.hpp"
#include "bagwiz/io/bag_io.hpp"

#include <sensor_msgs/msg/camera_info.hpp>

#include <rmw/types.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <span>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// Shared internals of the `cam-info` subcommands that round-trip typed
// sensor_msgs/msg/CameraInfo payloads through rmw (replace, recompute-p) plus
// the CameraInfo topic validation shared with `cam-info dump`. CLI-internal:
// this header lives with the command sources and is not installed.
namespace bagwiz::commands
{

// The message type every `cam-info` subcommand validates against.
inline constexpr const char * kCameraInfoType = "sensor_msgs/msg/CameraInfo";

// Most recent rmw error as a string, then reset so it does not leak into a
// later call. Returns a placeholder when no error state is set.
[[nodiscard]] std::string take_rmw_error();

// Deserialize one CameraInfo payload (the reader's CDR bytes) through rmw into
// a typed message. Throws std::runtime_error naming `topic` on failure.
[[nodiscard]] sensor_msgs::msg::CameraInfo deserialize_camera_info(
  std::span<const std::byte> payload, const rosidl_message_type_support_t * typesupport,
  const std::string & topic);

// Serialize a typed CameraInfo back to CDR bytes. Direct CdrWriter emission
// avoids the per-message rmw_serialize cost; the CameraInfo layout is fixed
// except for the variable-length D array, so it can be emitted in the ROS 2
// message definition's field order.
[[nodiscard]] std::vector<std::byte> serialize_camera_info(
  const sensor_msgs::msg::CameraInfo & msg);

// Names of every CameraInfo topic in `topics`, in the bag's topic order (for
// the "Available ... topic(s)" error listings).
[[nodiscard]] std::vector<std::string> camera_info_topic_names(
  std::span<const io::TopicInfo> topics);

// Outcome of validate_camera_info_targets().
struct CameraInfoTargets
{
  // Requested topics that passed validation, deduplicated while preserving
  // command-line order.
  std::vector<std::string> topics;
  bool all_valid = true;
};

// Validate each requested topic against a bag's topic list: it must exist and
// be a CameraInfo topic. Every failure is logged to `logger` (one line per bad
// topic, then the bag's available CameraInfo topics) so one run reports every
// bad topic; the caller must leave the bag untouched when all_valid is false.
[[nodiscard]] CameraInfoTargets validate_camera_info_targets(
  std::span<const io::TopicInfo> topics, const std::vector<std::string> & requested,
  const std::filesystem::path & input_path, const char * logger);

// Shared skeleton of the `cam-info replace` / `cam-info recompute-p`
// processors: every topic is routed through under its own name, and on a
// target topic each message is deserialized, mutated, re-serialized, and
// tallied. mutate() is the one genuinely command-specific step. The
// typesupport pointer is shared and read-only; transform() runs on the single
// producer thread, so scratch state (here and in derived classes) is never
// shared across threads.
class CameraInfoProcessor : public core::pipeline::Processor
{
public:
  CameraInfoProcessor(
    const std::vector<std::string> & topics, const rosidl_message_type_support_t * typesupport);

  // Number of messages rewritten on `topic` over the pass (0 for a topic that
  // carried none, or one not targeted). Read after the pipeline run completes.
  [[nodiscard]] std::uint64_t rewritten_count(const std::string & topic) const;

  [[nodiscard]] core::pipeline::Emit route(const std::string & in_topic) const override;

  [[nodiscard]] bool transforms() const override { return true; }

  [[nodiscard]] core::pipeline::TransformAction transform(
    const std::string & in_topic, std::span<const std::byte> in,
    std::vector<std::byte> & out) const override;

protected:
  // Overwrite the fields this command rewrites on one target-topic message,
  // between deserialize and serialize. Runs on the single producer thread (the
  // Processor contract), so derived-class scratch state needs no
  // synchronization. Throws std::runtime_error on failure.
  virtual void mutate(const std::string & in_topic, sensor_msgs::msg::CameraInfo & msg) const = 0;

private:
  std::unordered_set<std::string> targets_;
  const rosidl_message_type_support_t * typesupport_;
  // Per-target tally of rewritten messages. Mutable because transform() is const
  // per the Processor contract, which also guarantees transform() runs on the
  // single producer thread — so this is the sole writer and needs no
  // synchronization. Keys are fixed at construction (one per target topic);
  // transform() only updates existing values via at(), never inserts.
  mutable std::unordered_map<std::string, std::uint64_t> rewritten_;
};

}  // namespace bagwiz::commands

#endif  // COMMANDS__CAM_INFO_COMMON_HPP_
